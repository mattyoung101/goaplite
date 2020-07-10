# GOAPLite
by Matt Young, 2020.

This is my implementation of [Goal Oriented Action Planning](http://alumni.media.mit.edu/~jorkin/goap.html) (GOAP), 
a simplified STRIPS-like planning algorithm commonly use in games. 
GOAP allows agents to intelligently make long-term decisions based on a description of the actions they can perform, 
and a set of boolean variables describing their environment.

GOAPLite has been specifically designed for use in embedded systems, specifically our 
[RoboCup Jr Open Soccer robotics team](https://github.com/TeamOmicron).
With that in mind, it's written in pure C11 and designed with minimal overhead, simplicity and future-proofing in mind.

Currently, the planner uses a simple and inefficient depth first search algorithm. In future it could be rewritten to use
A* or one of its variants. 

Actions are currently loaded via a JSON file for ease of debugging, however, any other format
such as Protocol Buffers or a custom format could easily be added.

Partially inspired by this library: https://github.com/cpowell/cppGOAP

## Dependencies
All dependencies are bundled with the repository in the `lib` folder, unfortunately there are some. However, they are all
MIT licensed (cJSON, map) or public domain (DG_dynarr) so it shouldn't cause any licensing dillemas.

- [CJSON](https://github.com/DaveGamble/cJSON) to parse action description JSON documents. Can be removed if you have another way to declare these.
- [rxi's map](https://github.com/rxi/map) to store world state. Essential.
- [DG_dynarr](https://github.com/DanielGibson/Snippets/blob/master/DG_dynarr.h) for linked list implementation. Essential.

In future, I'm going to aim to implement these dependencies in GOAP code itself so you don't have to worry about dependencies.

## Compiling and running
This is a CMake project, so it should just be a matter of cloning the repo and doing the usual:
```
mkdir build
cd build
cmake-gui ..
make -j 4
./main
```
(or something along those lines).

The project is developed using CLion, so you can also just import it as a CLion project and run main.c

### Integrating in your own project
The way I would recommend integrating this into your own projects is simply copying `goap.c`, `goap.h` and the dependencies
in the lib folder somewhere into your source tree. It may be possible to build this as a shared or static library, but it's untested.

Once that's done, you should read `main.c` for a usage example on how to load an action list, set up a world state and
solve the problem.

## GOAP resources
- https://gamedevelopment.tutsplus.com/tutorials/goal-oriented-action-planning-for-a-smarter-ai--cms-20793
- http://alumni.media.mit.edu/~jorkin/goap.html
- https://en.wikipedia.org/wiki/Stanford_Research_Institute_Problem_Solver
- The PDFs in the docs folder in this repository

## Final notes
More documentation would be ideal, I agree. Especially Doxygen stuff, I'm going to get onto that one day.

Unfortunately, this project has come to somewhat of a standstill due to the fact that it's not really being used for anything
(putting it on our RoboCup Jr robot has sort of been stalled). If anyone has a use case for it, please let me know with a 
GitHub issue or email and I'd be more than happy to open it again.

Good luck!