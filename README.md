# GOAPLite
by Matt Young, 2020.

This is my implementation of [Goal Oriented Action Planning](http://alumni.media.mit.edu/~jorkin/goap.html) (GOAP), 
a simplified STRIPS-like planning algorithm commonly use in games. 
GOAP allows agents to intelligently make long-term decisions based on only a description of the actions they can perform, 
and a set of boolean variables describing their environment.

GOAPLite has been specifically designed for use in embedded systems, specifically our 
[RoboCup Jr Open Soccer robotics team](https://github.com/TeamOmicron).
With that in mind, it's written in pure C11 and designed with minimal overhead, simplicity and future-proofing in mind.

Currently, the planner uses a simple and inefficient depth first search algorithm. In future it will be rewritten to use
A* or one of its variants.

Partially inspired by this library: https://github.com/cpowell/cppGOAP

## Dependencies
All dependencies are bundled with the repository, unfortunately there are a few. 

- [CJSON](https://github.com/DaveGamble/cJSON) to parse action description JSON documents. Can be removed if you have another way to declare these.
- [rxi's map](https://github.com/rxi/map) to store world state. Essential.
- [DG_dynarr](https://github.com/DanielGibson/Snippets/blob/master/DG_dynarr.h) for linked list implementation. Essential.

## Compiling and running
TODO

### Integrating in your own project
TODO

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

## GOAP resources
- the docs folder in this repo
- https://gamedevelopment.tutsplus.com/tutorials/goal-oriented-action-planning-for-a-smarter-ai--cms-20793
- http://alumni.media.mit.edu/~jorkin/goap.html
- https://en.wikipedia.org/wiki/Stanford_Research_Institute_Problem_Solver

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