# GOAP library
by Matt Young.

This is my implementation of goal oriented action planning (GOAP), an simplified STRIPS-like planner but significantly
more advanced than our current AI techniques (we currently use FSMs). It's written in C11 and designed with minimal overhead,
simplicity and future-proofing in mind.

Inspiration from this: https://github.com/cpowell/cppGOAP

## Requirements
- Uses minimal resources, including for dependencies, since this is running on the ESP32
- Easy to declare GOAP action lists, in whatever way this is most feasible (including potentially external data)
- Easy to execute GOAP plans (just like how it's easy to execute FSM plans currently)

## Dependencies
- linked list for storing list of actions/world states: DG_dynarr.h
- hashmap with string keys: map.c/map.h from rxi
- JSON parser for loading GOAP config from disk: cJSON
- AVL tree for implementing Dijkstra's algorithm
- TODO: in future, rewrite all our dependencies ourselves to save external libraries (maybe except JSON parsing)

## API design
### FIXME outdated
### `goap_action_status_t`
- DONE
- RUNNING
- FAILED

### `goap_action_t`
#### Variables
- `char *name`
- `uint32_t cost` (negative costs are forbidden)
- `map_bool_t *preconditions`
- `map_bool_t *postconditions`
- function pointer to: `goap_action_status_t actionCode(void);`
#### Functions
- use the hashmap API directly to set preconditions/postconditions
- use the struct itself to set name and cost
- eventually we'll support deserialising these from JSON and protobuf

### `goap_worldstate_t`
Just a map with key string, value bool.

### `goap_actionlist_t`
Just a linked list of `goap_action_t` pointers.

### `goap_planner_t`
#### Variables
- `goap_actionlist_t actions` (type of list is pointer to goap_action_t)
- `goap_worldstate_t currentState`
- `goap_world_state_t goalState`
#### Functions
- `goap_planner_t *goap_planner_new()`
- `goap_planner_free()`
- `goap_actionlist_t goap_planner_plan()` (what if no plan can be generated??)

## Workflow on robot
- Set goal state in setup
- Loop:
    - Collect sensor data
    - Update GOAP current state with new data
    - Is current GOAP state different?
        - Yes: generate new plan. Old plan would now probably be invalid.
    - Is current action complete?
        - Yes: did this action fail?
            - Yes: generate new plan from current world state
            - No: continue to next action in plan if not last action
        - No: continue executing current action 