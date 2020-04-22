#include "goap.h"
#include <stdio.h>
#include "cJSON.h"
#include "log/log.h"

/**
 * This heuristic function should be admisable because it never overestimates the cost of reaching the goal
 * Based on the cppGOAP heuristic (not the GPGOAP one)
 */
static void heuristic(goap_worldstate_t a, goap_worldstate_t b){
    // difference between two states hashmaps
    map_iter_t aIter = map_iter(a);
    map_iter_t bIter = map_iter(b);
}

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

/** returns the list of actions that can be executed after executing the given action. assumes the action can be executed. */
static goap_actionlist_t execute_find_neighbours(goap_action_t action, goap_actionlist_t actions, goap_worldstate_t world){
    goap_actionlist_t neighbours = {0};
    goap_worldstate_t newWorld = execute_action(action, world);

    // now iterate over all actions and see which ones we can apply in our new world after performing the action
    for (goap_action_t *it = da_begin(actions), *end = da_end(actions); it != end; ++it){
        if (can_perform_action(*it, newWorld)){
            da_add(neighbours, *it);
        }
    }

    map_deinit(&newWorld);
    return neighbours;
}

/** returns the list of actions that can be executed given the current world state */
static goap_actionlist_t find_neighbours(goap_actionlist_t actions, goap_worldstate_t world, goap_action_t currentAction){
    goap_actionlist_t neighbours = {0};

    // now iterate over all actions and see which ones we can apply in our new world after performing the action
    for (goap_action_t *it = da_begin(actions), *end = da_end(actions); it != end; ++it){
        if (strcmp(it->name, currentAction.name) != 0 && can_perform_action(*it, world)){
            da_add(neighbours, *it);
        }
    }

    return neighbours;
}

/** list of goap_action_t pointers */
DA_TYPEDEF(goap_action_t*, actionlistp_t)

/** used for internal debugging only */
static void dump_set(map_bool_t set){
    printf("dump_set() %d entries:\n", set.base.nnodes);
    map_iter_t iter = map_iter();
    const char *key = NULL;
    while ((key = map_next(&set, &iter))) {
        printf("\t- %s\n", key);
    }
}

goap_actionlist_t goap_planner_plan(goap_worldstate_t currentWorld, goap_worldstate_t goal, goap_actionlist_t allActions){
    log_trace("GOAP planner working with %zu actions", da_count(allActions));
    goap_actionlist_t plan = {0};

    // FIXME is it a good idea to iterate from start to beginning?

    // check if we're already at the goal for some reason
    if (goap_worldstate_compare(currentWorld, goal)){
        log_warn("Goal state is already satisfied, no planning required");
        return plan;
    }

    // here we basically make the equivalent of a set using a hash table, which is a little bit tricky but should be fine
    goap_actionlist_t vertexSet = {0};
    map_bool_t vertexChecker = {0};

    // Dijkstra's setup: reset all nodes, add whichever ones we can access right now to our vertex queue
    for (goap_action_t *it = da_begin(allActions), *end = da_end(allActions); it != end; ++it){
        it->_dist = UINT32_MAX;
        it->_prev = NULL;
        if (can_perform_action(*it, currentWorld)){
            da_add(vertexSet, *it);
            map_set(&vertexChecker, it->name, true);
        }
    }

    // find list of actions we can perform in the current world state
    puts("\nInitial available actions:");
    dump_set(vertexChecker);
    if (da_count(vertexSet) == 0){
        log_error("No actions can be performed in the current world state!");
        da_free(vertexSet);
        map_deinit(&vertexChecker);
        return plan;
    }
    // set our source node to 0 distance
    da_getptr(vertexSet, 0)->_dist = 0;
    printf("Source is: %s\n", da_get(vertexSet, 0).name);
    char *sourceName = da_get(vertexSet, 0).name;

    // use Dijkstra's algorithm to find path to goal state
    // based on pseudocode: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm#Pseudocode
    // TODO don't we need to recursively call Dijkstra's on each first available action?
    // since they could lead to different outcomes, one of which may not be possible to have a solution to the goal!
    // we're going to ignore this for now, just so I can get a working prototype out the door, we'll fix later
    while (da_count(vertexSet) > 0){
        // locate smallest element in queue
        goap_action_t u = {0};
        size_t smallestIdx = 0;
        uint32_t smallestDist = UINT32_MAX;

        for (size_t i = 0; i < da_count(vertexSet); i++){
            goap_action_t action = da_get(vertexSet, i);
            if (action._dist <= smallestDist){
                u = action;
                smallestDist = action._dist;
                smallestIdx = i;
            }
        }
        printf("located smallest item %s at index %zu with distance %u\n", u.name, smallestIdx, smallestDist);

        // let's pretend we executed the action and see what our world looks like now
        printf("Going to execute action: %s\n", u.name);
        goap_worldstate_t newWorld = execute_action(u, currentWorld);
        printf("World after executing %s:\n", u.name);
        goap_worldstate_dump(newWorld);

        // check if we've reached the goal and return path
        if (goap_worldstate_compare(newWorld, goal)){
            if (u._prev != NULL || strcmp(u.name, sourceName) == 0){
                printf("Hooray! We reached the goal! Reconstrucing path...\n");
                printf("current aciton is: %s\n", u.name);
                printf("i bet this will crash, previous node name is: %s\n", u._prev->name);
                break;
            } else {
                printf("ignoring invalid solution to path");
            }
        }

        // visit all neighbours that we can execute given our last world state
        goap_actionlist_t neighbours = find_neighbours(allActions, newWorld, u);
        printf("We have %zu neighbours\n", da_count(neighbours));
        for (goap_action_t *v = da_begin(neighbours), *end = da_end(neighbours); v != end; ++v){
            printf("Visiting neighbour of %s called %s\n", u.name, v->name);
            // note: if it's a neighbour, it means we can DIRECTLY transition into this node from our current node
            // so therefore we can assume we can reach it, and length(u, v) should just be v.cost??
            uint32_t alt = u._dist + v->cost;
            if (alt < v->_dist){
                v->_dist = alt;
                v->_prev = &u; // TODO THIS IS BUGGED BECAUSE U IS STACK ALLOCATED!! FIX MEEEEEEE
                // we've found a new neighbour so we should probably add it to the queue TODO if it's not already in there
                if (map_get(&vertexChecker, v->name) == NULL){
                    printf("Neighbour is NOT in queue, adding it");
                    da_add(vertexSet, *v);
                    map_set(&vertexChecker, v->name, true);
                }
            }
        }

        // remove element from queue, we can already use "u" to reference it
        da_delete(vertexSet, smallestIdx);
        map_remove(&vertexChecker, u.name);

        da_free(neighbours);
    }

    da_free(vertexSet);
    map_deinit(&vertexChecker);
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
    printf("goap_actionlist_dump() %zu entries:\n", da_count(list));
    for (size_t i = 0; i < da_count(list); i++){
        printf("\t%zu. %s\n", i + 1, da_get(list, i).name);
    }
}

void goap_worldstate_dump(goap_worldstate_t world){
    printf("goap_worldstate_dump() %d entries:\n", world.base.nnodes);
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