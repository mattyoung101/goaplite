#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "DG_dynarr.h"
#include "map.h"
#include "avl.h"

typedef map_t(bool) map_bool_t;
/** Used to define the current state of a GOAP world */
typedef map_bool_t goap_worldstate_t;

typedef enum {
    /** The action has not yet been completed and is still running */
    GOAP_STATUS_RUNNING = 0,
    /** The action has completed successfully */
    GOAP_STATUS_SUCCESS,
    /** The action has failed and a new plan should be generated */
    GOAP_STATUS_FAILED
} goap_action_status_t;

typedef struct goap_action_t goap_action_t;
typedef struct goap_action_t {
    char *name;
    uint32_t cost;
    /** a hashmap of conditions that must be true for this action to be executed */
    map_bool_t preConditions;
    /** a hashmap of changes to the world state that will be set once this action completes */
    map_bool_t postConditions;
    /** code that is executed while the action is running, returns the status (running, done or failed) */
    goap_action_status_t (*actionCode)(void);

    // internal used for pathfinding only
    bool _discovered;
    struct goap_action_t *_prev;
    uint32_t _dist;
} goap_action_t;

/** A linked list of goap_action_t items */
DA_TYPEDEF(goap_action_t, goap_actionlist_t)

/**
 * Calculates the optimal plan from the current state to the goal state using a pathfinding algorithm, currently A*.
 * If no plan could be created, prints an error and returns an empty list.
 *
 * The user is responsible for allocating and freeing all parameters from this function, so they can also handle
 * threading/multi-core synchronisation if necessary
 * @param current the current GOAP state allocated by the user
 * @param goal the goal world state allocated by the user
 * @param allActions the list of actions available to the planner, try goap_parse_* to generate this
 * @returns if a successful plan was generated, an ordered linked list of the actions in the plan, otherwise an empty list.
 * The user must free this list with a call to da_free() but ABSOLUTELY NOT a call to goap_actionlist_free() or
 * double frees will occur.
 */
goap_actionlist_t goap_planner_plan(goap_worldstate_t currentWorld, goap_worldstate_t goal, goap_actionlist_t allActions);

/**
 * Generates a goap_actionlist_t by deserialising a JSON document.
 *
 * NOTE: You must call goap_actionlist_free() to delete this list and its contents properly.
 *
 * NOTE: currently the goal state is not parsed, you must create this manually
 * @param str the contents of the JSON document
 */
goap_actionlist_t goap_parse_json(char *str, size_t length);
/** TODO implement this */
goap_actionlist_t goap_parse_protobuf(void);
/** Free all resources associated with the given action list */
void goap_actionlist_free(goap_actionlist_t *list);
/** Dumps a goap_actionlist_t to the console */
void goap_actionlist_dump(goap_actionlist_t list);
/** Compares two world states, returning true if and only if they are exactly identical (including same number of keys) */
bool goap_worldstate_compare_strict(goap_worldstate_t a, goap_worldstate_t b);
/** Compares two world states and returns true if they're functionally equivalent, ignoring extraneous keys */
bool goap_worldstate_compare(goap_worldstate_t currentState, goap_worldstate_t goal);
/** Dumps a goap_worldstate_t to the console */
void goap_worldstate_dump(goap_worldstate_t world);