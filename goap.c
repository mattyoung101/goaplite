#include "goap.h"
#include <stdio.h>
#include "cJSON.h"
#include "log/log.h"

#define ITERATE_ARRAY(array) goap_action_t *it = da_begin(array), *end = da_end(array); it != end; ++it
#define ITERATE_ARRAYPTR(array) goap_action_t **it = da_begin(array), **end = da_end(array); it != end; ++it

/** returns true if the given action can be executed in the current world state */
static bool can_perform_action(goap_action_t action, goap_worldstate_t world){
    return goap_worldstate_compare(world, action.preConditions);
}

/** returns the result of executing the action on the current world. assumes the action can be executed. map_deinit() the result. */
static goap_worldstate_t execute_action(goap_action_t action, goap_worldstate_t world){
    goap_worldstate_t copy = world;
    map_iter_t iter = map_iter(&action.postConditions);
    const char *key = NULL;

    // let's pretend we executed the action, so apply our post-conditions to the backup world
    while ((key = map_next(&action.postConditions, &iter))){
        bool *postResult = map_get(&action.postConditions, key);
        map_set(&copy, key, *postResult);
    }

    return copy;
}

/** checks if any of the parents of "potential" are equal to "parent" */
static bool contains_parent(goap_action_t potential, goap_action_t parent){
    if (potential._parent == NULL) return false; // has no parents

    goap_action_t *it = potential._parent;
    while (it != NULL){
        // check if they're equal by names
        if (strcmp(it->name, parent.name) == 0){
            return true;
        }
        it = it->_parent;
    }
    return false;
}

/** returns the list of actions that can be executed from this action given the current world state */
static goap_actionlist_t find_neighbours(goap_actionlist_t actions, goap_worldstate_t world, goap_action_t currentAction){
    goap_actionlist_t neighbours = {0};
    for (ITERATE_ARRAY(actions)){
        // exclude actions which are ourselves, that we cannot perform and that contain a parent who is currentActions
        if (can_perform_action(*it, world) && strcmp(it->name, currentAction.name) != 0 && !contains_parent(*it, currentAction)){
            da_add(neighbours, *it);
        }
    }
    return neighbours;
}

/** list of goap_action_t pointers */
DA_TYPEDEF(goap_action_t*, actionlistptr_t)
/** list of action lists */
DA_TYPEDEF(goap_actionlist_t, actionlist2_t)

/** a node used for graph searching */
typedef struct {
    goap_actionlist_t parents;
    goap_worldstate_t currentState;
    goap_action_t currentAction;
} node_t;

goap_actionlist_t goap_planner_plan(goap_worldstate_t currentWorld, goap_worldstate_t goal, goap_actionlist_t allActions){
    log_trace("GOAP planner working with %zu actions", da_count(allActions));
    goap_actionlist_t plan = {0};

    // check if we're already at the goal for some reason
    if (goap_worldstate_compare(currentWorld, goal)){
        log_warn("Goal state is already satisfied, no planning required");
        return plan;
    }

    // so here's my current thinking, the problem with Dijkstra is either my implemntation's wrong, or it needs the
    // entire graph pre-planned in memory. although probably my code is just bugged, if it needs the whole graph in memory
    // then we have to traverse it with a BFS/DFS. and if we're doing that, well, we may as well record all paths that
    // got us to the finish and find the one with the least cost (or shortest length?)
    // however, I think the real problem with my Dijkstra implementation is that each node MUST NOT visit any of its parents
    // although we restricted a node from visiting itself, it could visit its parents which caused infinite loops
    // idk if that's the real fix though, so in the mean time, I'm going to use a DFS
    goap_actionlist_t stack = {0};
    actionlistptr_t garbage = {0};

    // reset all nodes, and populate list of actions we can perform given the current world state
    goap_action_t firstAction = {0};
    for (goap_action_t *it = da_begin(allActions), *end = da_end(allActions); it != end; ++it){
        it->_parent = NULL;
        if (can_perform_action(*it, currentWorld)){
            firstAction = *it;
            //da_add(stack, *it);
        }
    }
    da_add(stack, firstAction);

    puts("\nInitial available actions:");
    goap_actionlist_dump(stack);
    if (da_count(stack) == 0){
        log_error("No actions can be performed in the current world state!");
        da_free(stack);
        return plan;
    }

    // TODO need to keep track of world state as we go otherwise we can't handle more than one precondition
    // TODO possibly trade pointer to parent for a list of parents (might cause less memory bugs)

    while (da_count(stack) > 0){
        printf("\nStack has %zu elements\n", da_count(stack));

        // pop the last element off the stack, we can't use da_pop() because we need a pointer to it
        goap_action_t *node = da_getptr(stack, da_count(stack) - 1);
        da_deletefast(stack, da_count(stack) - 1);
        printf("Visiting node: %s\n", node->name);

        // let's pretend we executed the action and see what our world looks like now
        goap_worldstate_t newWorld = execute_action(*node, currentWorld);
        printf("World after executing %s:\n", node->name);
        goap_worldstate_dump(newWorld);

        // check if goal and reconstruct path, calculate score and add to list of paths
        if (goap_worldstate_compare(newWorld, goal)){
            printf("Reached goal!\n");
            // TODO reconstruct path and cache score
            break;
        }

        // clone this action onto the heap (otherwise things break in very bad ways)
        goap_action_t *clone = malloc(sizeof(goap_action_t));
        memcpy(clone, node, sizeof(goap_action_t));

        // if not goal: explore children, set parents and add to queue
        goap_actionlist_t neighbours = find_neighbours(allActions, newWorld, *node);
        for (ITERATE_ARRAY(neighbours)){
            printf("Visiting neighbour of %s: %s\n", node->name, it->name);
            it->_parent = clone;
            da_add(stack, *it);
        }
    }

    // free any extraneous actions we allocated during the search
    for (ITERATE_ARRAYPTR(garbage)){
        printf("Freeing allocated node: %s\n", (*it)->name);
        free(*it);
    }

    da_free(garbage);
    da_free(stack);
    return plan;
}

goap_actionlist_t goap_parse_json(char *str, size_t length){
    log_trace("Using cJSON v%s", cJSON_Version());
    cJSON *json = cJSON_ParseWithLength(str, length);
    goap_actionlist_t out = {0};
    if (json == NULL){
        log_error("Failed to parse JSON document: token %s\n", cJSON_GetErrorPtr());
        goto finish;
    }

    cJSON *actions = cJSON_GetObjectItem(json, "actions");
    if (!cJSON_IsArray(actions)){
        log_error("Invalid JSON document: actions array is not an array, or doesn't exist");
        goto finish;
    }

    cJSON *action = NULL;
    cJSON_ArrayForEach(action, actions){
        cJSON *name = cJSON_GetObjectItem(action, "name");
        cJSON *cost = cJSON_GetObjectItem(action, "cost");
        cJSON *preConditions = cJSON_GetObjectItem(action, "preConditions");
        cJSON *postConditions = cJSON_GetObjectItem(action, "postConditions");
        char *dump = cJSON_Print(action);

        // validate our parsed data
        if (!cJSON_IsString(name)){
            log_error("Invalid JSON object: action name is not a string or doesn't exist\n%s", dump);
            free(dump);
            break;
        } else if (!cJSON_IsNumber(cost)){
            log_error("Invalid JSON object: action cost is not a number or doesn't exist\n%s", dump);
            free(dump);
            break;
        } else if (!cJSON_IsObject(preConditions)){
            log_error("Invalid JSON object: action preConditions is not an object or doesn't exist\n%s", dump);
            free(dump);
            break;
        } else if (!cJSON_IsObject(postConditions)){
            log_error("Invalid JSON object: action postConditions is not an object or doesn't exist\n%s", dump);
            free(dump);
            break;
        } else {
            log_trace("Verification passed for config object");
            free(dump);
        }
        // additional requirements that are not checked here:
        // - each action MUST have a unique string name

        // now that we've got a valid document, serialise the JSON into a GOAP action
        goap_action_t parsedAction = {0};
        // because cJSON_Delete() apparently also deletes all the strings we have to strdup() the name
        parsedAction.name = strdup(name->valuestring);
        parsedAction.cost = cost->valueint;
        map_init(&parsedAction.preConditions);
        map_init(&parsedAction.postConditions);

        // apparently this works for objects to
        cJSON *preItem = NULL;
        cJSON_ArrayForEach(preItem, preConditions){
            map_set(&parsedAction.preConditions, preItem->string, preItem->valueint);
        }
        cJSON *postItem = NULL;
        cJSON_ArrayForEach(postItem, postConditions){
            map_set(&parsedAction.postConditions, postItem->string, postItem->valueint);
        }

        // TODO we'd also need to set the function pointers based on name here or later somewhere else
        da_add(out, parsedAction);
    }

    finish:
    cJSON_Delete(json);
    return out;
}

void goap_actionlist_free(goap_actionlist_t *list){
    for (size_t i = 0; i < da_count(*list); i++){
        goap_action_t action = da_get(*list, i);
        free(action.name);
        map_deinit(&action.preConditions);
        map_deinit(&action.postConditions);
        // action itself is stack allocated (or something like that) so no need to free it
    }
    da_free(*list);
}

void goap_actionlist_dump(goap_actionlist_t list){
    //printf("goap_actionlist_dump() %zu entries:\n", da_count(list));
    for (size_t i = 0; i < da_count(list); i++){
        printf("\t%zu. %s\n", i + 1, da_get(list, i).name);
    }
}

void goap_worldstate_dump(goap_worldstate_t world){
    //printf("goap_worldstate_dump() %d entries:\n", world.base.nnodes);
    map_iter_t iter = map_iter();
    const char *key = NULL;
    while ((key = map_next(&world, &iter))) {
        printf("\t%s: %s\n", key, *map_get(&world, key) ? "true" : "false");
    }
}

bool goap_worldstate_compare_strict(goap_worldstate_t a, goap_worldstate_t b){
    // TODO this function may be broke, should rewrite
    // must have same number of keys
    if (a.base.nnodes != b.base.nnodes) return false;
    map_iter_t iter = map_iter();

    const char *key = NULL;
    while ((key = map_next(&a, &iter))) {
        bool *aValue = map_get(&a, key);
        bool *bValue = map_get(&b, key);

        // one of the maps does not contain the key
        if (aValue == NULL || bValue == NULL || *aValue != *bValue){
            return false;
        }
    }
    return true;
}

bool goap_worldstate_compare(goap_worldstate_t currentState, goap_worldstate_t goal){
    map_iter_t iter = map_iter();
    const char *key = NULL;

    while ((key = map_next(&goal, &iter))){
        bool *curVal = map_get(&currentState, key);
        bool *targetVal = map_get(&goal, key);

        if (curVal == NULL || targetVal == NULL || *curVal != *targetVal){
            return false;
        }
    }
    return true;
}