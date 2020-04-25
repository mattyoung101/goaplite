#include <stdio.h>
#include <errno.h>
#include <time.h>

#define DG_DYNARR_IMPLEMENTATION
#include "DG_dynarr.h"
#undef DG_DYNARR_IMPLEMENTATION
#include "goap.h"
#include "log/log.h"

static goap_action_status_t uselessActionFunction(){
    printf("Executing useless action function\n");
    return GOAP_STATUS_SUCCESS;
}

/** util function to return a null terminated string from a file on disk */
static char *utils_load_file(char *path, long *size){
    char *buf = NULL;
    long length;
    FILE *f = fopen(path, "rb");
    if (!f){
        fprintf(stderr, "Failed to open file \"%s\": %s\n", path, strerror(errno));
        *size = 0;
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    length = ftell(f);
    *size = length;
    fseek(f, 0, SEEK_SET);
    buf = malloc(length + 1);
    fread(buf, 1, length, f);
    fclose(f);
    buf[length] = '\0'; // null terminate it
    return buf;
}

/** qsort comparator that can be used to terribly shuffle an array for testing only */
static int random_comparator(const void *a, const void *b){
    return rand() % 3 - 1;
}

int main() {
    log_set_level(LOG_TRACE);
    long jsonSize = 0;
    char *jsonStr = utils_load_file("../goap2.json", &jsonSize);
    srand(time(NULL));

    // load actions from config
    goap_actionlist_t parsedActions = goap_parse_json(jsonStr, jsonSize);
    if (da_count(parsedActions) == 0){
        log_error("Parse error!");
        return EXIT_FAILURE;
    }

    // assign action functions, in the real robot this would be more in depth
    for (goap_action_t *it = da_begin(parsedActions), *end = da_end(parsedActions); it != end; ++it){
        it->actionFunction = uselessActionFunction;
    }

    // shuffle our actions list for an unbiased test
    da_sort(parsedActions, random_comparator);
    puts("Parsed action list:");
    goap_actionlist_dump(parsedActions);

    // setup current and goal state
    goap_worldstate_t currentState = {0};
    goap_worldstate_t goalState = {0};
    map_set(&currentState, "Awake", false);
    //map_set(&currentState, "Employed", true);

    map_set(&goalState, "Employed", true);
    map_set(&goalState, "Happy", true);
    map_set(&goalState, "Awake", false);
    map_set(&goalState, "Clean", true);

    // generate plan
    goap_actionlist_t plan = goap_planner_plan(currentState, goalState, parsedActions);
    puts("\nPlan:");
    goap_actionlist_dump(plan);

    // cleanup
    fflush(stdout);
    da_free(plan);
    map_deinit(&currentState);
    map_deinit(&goalState);
    goap_actionlist_free(&parsedActions);
    free(jsonStr);
    return EXIT_SUCCESS;
}
