/*
 * Copyright (c) 2020 Matt Young.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "goap.h"
#include <stdio.h>
#include "cJSON.h"

#define ACTIONLIST_ITER(array) goap_action_t *it = da_begin(array), *end = da_end(array); it != end; ++it
#define NODELIST_ITER(array) node_t *it = da_begin(array), *end = da_end(array); it != end; ++it

/** a node used for graph searching */
typedef struct {
    goap_actionlist_t parents;
    /** current world state at this node (sum of all parents world states, basically) */
    goap_worldstate_t worldState;
    /** total cost of this node so far */
    uint32_t cost;
} node_t;

DA_TYPEDEF(node_t, nodelist_t)

/** returns true if the given action can be executed in the current world state */
static bool can_perform_action(goap_action_t action, goap_worldstate_t world) {
    return goap_worldstate_compare(world, action.preConditions);
}

/** updates the specified world state by applying the post conditions of the specified action. works "in place" on world */
static void execute_action(goap_action_t action, goap_worldstate_t *world) {
    map_iter_t iter = map_iter();
    const char *key = NULL;
    // let's pretend we executed the action, so apply our post-conditions to the backup world
    while ((key = map_next(&action.postConditions, &iter))) {
        bool *postResult = map_get(&action.postConditions, key);
        map_set(world, key, *postResult);
    }
}

/** checks if the given action list contains an entry with the name of "name" */
static bool contains_name(char *name, goap_actionlist_t history) {
    for (ACTIONLIST_ITER(history)) {
        if (strcmp(it->name, name) == 0) {
            return true;
        }
    }
    return false;
}

/** returns the list of actions that can be executed from this node given it's parents and current state */
static goap_actionlist_t find_executable_actions(node_t node, goap_actionlist_t actions) {
    goap_actionlist_t neighbours = {0};
    for (ACTIONLIST_ITER(actions)) {
        // exclude actions that we cannot execute or that are our parents
        if (can_perform_action(*it, node.worldState) && !contains_name(it->name, node.parents)) {
            da_add(neighbours, *it);
        }
    }
    return neighbours;
}

/** clones the specified map to a new map w/o freeing the old map */
static goap_worldstate_t map_clone(goap_worldstate_t oldMap) {
    goap_worldstate_t newMap = {0};
    map_iter_t iter = map_iter();
    const char *key = NULL;
    while ((key = map_next(&oldMap, &iter))) {
        bool *val = map_get(&oldMap, key);
        map_set(&newMap, key, *val);
    }
    return newMap;
}

/** clones the specified list to a new list w/o freeing the old list s*/
static goap_actionlist_t list_clone(goap_actionlist_t oldList) {
    goap_actionlist_t newList = {0};
    for (ACTIONLIST_ITER(oldList)) {
        da_add(newList, *it);
    }
    return newList;
}

/** used for sorting */
static int cost_comparator(const void *a, const void *b){
    node_t *nodeA = (node_t*) a;
    node_t *nodeB = (node_t*) b;

    if (nodeA->cost == nodeB->cost){
        // if tie, choose path length
        if (da_count(nodeA->parents) == da_count(nodeB->parents)){
            return 0;
        } else if (da_count(nodeA->parents) > da_count(nodeB->parents)){
            return 1;
        } else {
            return -1;
        }
    } else if (nodeA->cost > nodeB->cost){
        return 1;
    } else {
        return -1;
    }
}

goap_actionlist_t goap_planner_plan(goap_worldstate_t currentWorld, goap_worldstate_t goal, goap_actionlist_t allActions) {
    printf("GOAP planner working with %zu actions", da_count(allActions));
    goap_actionlist_t plan = {0};

    // check if we're already at the goal for some reason
    if (goap_worldstate_compare(currentWorld, goal)) {
#if GOAP_DEBUG
        puts("Goal state is already satisfied, no planning required");
#endif
        return plan;
    }

    // use a depth first search to iterate over the whole graph, in future use A*/Dijkstra
    nodelist_t stack = {0};
    nodelist_t solutions = {0};

    // add our current state to the stack
    node_t initial = {0};
    initial.worldState = map_clone(currentWorld);
    da_add(stack, initial);
    uint32_t count = 0;

    while (da_count(stack) > 0) {
        printf("\nStack has %zu elements\n", da_count(stack));
        // pop the last element off the stack, we can copy it since we're throwing it away
        node_t node = da_pop(stack);
        count++;

        // (just debug stuff)
        printf("Visiting node with %zu parents\n", da_count(node.parents));
        if (da_count(node.parents) > 0) {
            printf("Parents are:\n");
            goap_actionlist_dump(node.parents);
        }
        printf("World state of this node is:\n");
        goap_worldstate_dump(node.worldState);

        // let's see what actions we can execute in the current world state of the node
        goap_actionlist_t neighbours = find_executable_actions(node, allActions);
        printf("List of actions we can perform from this state:\n");
        goap_actionlist_dump(neighbours);

        // iterate through each action and put a new node on the search list
        for (ACTIONLIST_ITER(neighbours)) {
            // clone map (so we don't cause annoying heap use after free problems) and pretend we executed the action
            goap_worldstate_t newWorld = map_clone(node.worldState);
            execute_action(*it, &newWorld);
            printf("After performing %s, new world state is:\n", it->name);
            goap_worldstate_dump(newWorld);

            // clone the list as well and add the considered neighbour to it
            goap_actionlist_t parentsClone = list_clone(node.parents);
            da_add(parentsClone, *it);

            // make a new node with the updated data
            node_t newNode = {0};
            newNode.parents = parentsClone;
            newNode.worldState = newWorld;
            newNode.cost = node.cost + it->cost;

            // decide which list we add our node to
            if (goap_worldstate_compare(newWorld, goal)) {
                printf("Reached goal! Adding to solutions list\n");
                da_add(solutions, newNode);
            } else {
                printf("Added new node with %zu parents to stack\n", da_count(newNode.parents));
                da_add(stack, newNode);
            }
        }

        // free the node's contents now that we no longer need it
        map_deinit(&node.worldState);
        da_free(node.parents);
        da_free(neighbours);
    }
    printf("Search is complete. Visited %u nodes, found %zu solutions\n\n", count, da_count(solutions));

    // check for no solutions
    if (da_count(solutions) == 0){
#if GOAP_DEBUG
        fprintf(stderr, "No solutions found in search!");
#endif
        da_free(solutions);
        da_free(stack);
        return plan;
    }

    // sort solutions by least cost
    da_sort(solutions, cost_comparator);
    node_t bestSolution = da_get(solutions, 0);
#if GOAP_DEBUG
    printf("Best solution: cost %u, length %zu:\n", bestSolution.cost, da_count(bestSolution.parents));
#endif
    goap_actionlist_dump(bestSolution.parents);

    // add the solution to the output array
    for (ACTIONLIST_ITER(bestSolution.parents)){
        da_add(plan, *it);
    }

    // free up all resources we allocated
    for (NODELIST_ITER(solutions)){
        map_deinit(&it->worldState);
        da_free(it->parents);
    }
    da_free(solutions);
    da_free(stack);
    return plan;
}

goap_actionlist_t goap_parse_json(char *str, size_t length) {
    cJSON *json = cJSON_ParseWithLength(str, length);
    goap_actionlist_t out = {0};
    if (json == NULL) {
#if GOAP_DEBUG
        fprintf(stderr, "Failed to parse JSON document: token %s\n", cJSON_GetErrorPtr());
#endif
        goto finish;
    }

    cJSON *actions = cJSON_GetObjectItem(json, "actions");
    if (!cJSON_IsArray(actions)) {
#if GOAP_DEBUG
        fprintf(stderr, "Invalid JSON document: actions array is not an array, or doesn't exist");
#endif
        goto finish;
    }

    cJSON *action = NULL;
    cJSON_ArrayForEach(action, actions) {
        cJSON *name = cJSON_GetObjectItem(action, "name");
        cJSON *cost = cJSON_GetObjectItem(action, "cost");
        cJSON *preConditions = cJSON_GetObjectItem(action, "preConditions");
        cJSON *postConditions = cJSON_GetObjectItem(action, "postConditions");
        char *dump = cJSON_Print(action);

        // validate our parsed data
        if (!cJSON_IsString(name)) {
#if GOAP_DEBUG
            fprintf(stderr, "Invalid JSON object: action name is not a string or doesn't exist\n%s", dump);
#endif
            free(dump);
            break;
        } else if (!cJSON_IsNumber(cost)) {
#if GOAP_DEBUG
            fprintf(stderr, "Invalid JSON object: action cost is not a number or doesn't exist\n%s", dump);
#endif
            free(dump);
            break;
        } else if (!cJSON_IsObject(preConditions)) {
#if GOAP_DEBUG
            fprintf(stderr, "Invalid JSON object: action preConditions is not an object or doesn't exist\n%s", dump);
#endif
            free(dump);
            break;
        } else if (!cJSON_IsObject(postConditions)) {
#if GOAP_DEBUG
            fprintf(stderr, "Invalid JSON object: action postConditions is not an object or doesn't exist\n%s", dump);
#endif
            free(dump);
            break;
        } else {
#if GOAP_DEBUG
            puts("Verification passed for config object");
#endif
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
        cJSON_ArrayForEach(preItem, preConditions) {
            map_set(&parsedAction.preConditions, preItem->string, preItem->valueint);
        }
        cJSON *postItem = NULL;
        cJSON_ArrayForEach(postItem, postConditions) {
            map_set(&parsedAction.postConditions, postItem->string, postItem->valueint);
        }

        da_add(out, parsedAction);
    }

    finish:
    cJSON_Delete(json);
    return out;
}

void goap_actionlist_free(goap_actionlist_t *list) {
    for (size_t i = 0; i < da_count(*list); i++) {
        goap_action_t action = da_get(*list, i);
        free(action.name);
        map_deinit(&action.preConditions);
        map_deinit(&action.postConditions);
        // action itself is stack allocated (or something like that) so no need to free it
    }
    da_free(*list);
}

void goap_actionlist_dump(goap_actionlist_t list) {
    //printf("goap_actionlist_dump() %zu entries:\n", da_count(list));
    if (da_count(list) == 0) {
        printf("\t(empty action list)\n");
        return;
    }

    for (size_t i = 0; i < da_count(list); i++) {
        printf("\t%zu. %s\n", i + 1, da_get(list, i).name);
    }
}

void goap_worldstate_dump(goap_worldstate_t world) {
    //printf("goap_worldstate_dump() %d entries:\n", world.base.nnodes);
    map_iter_t iter = map_iter();
    const char *key = NULL;
    uint32_t count = 0;
    while ((key = map_next(&world, &iter))) {
        printf("\t%s: %s\n", key, *map_get(&world, key) ? "true" : "false");
        count++;
    }

    if (count == 0) {
        printf("\t(empty world state)\n");
    }
}

bool goap_worldstate_compare(goap_worldstate_t currentState, goap_worldstate_t goal) {
    map_iter_t iter = map_iter();
    const char *key = NULL;

    while ((key = map_next(&goal, &iter))) {
        bool *curVal = map_get(&currentState, key);
        bool *targetVal = map_get(&goal, key);

        // interesting to note: STRIPS planners like GOAP make the closed world assumption, which we see on this line.
        // if curVal is NULL, it means that the key from the goal state was not found in the current state.
        // we assume this means it's false, but in fact it's just "unknown" - it could, in fact, be true
        if (curVal == NULL || targetVal == NULL || *curVal != *targetVal) {
            return false;
        }
    }
    return true;
}