# cmake/stamp_git.cmake — A-5: build provenance that is actually current.
#
# Run with `cmake -P` from a custom target that executes on EVERY build, not at
# configure time.
#
# ---------------------------------------------------------------------------
# THE DEFECT THIS EXISTS FOR
# ---------------------------------------------------------------------------
# CMakeLists.txt stamped SAT_GIT_HASH with `execute_process` at CONFIGURE time
# and injected it as a compile definition. CMake does not re-configure because
# HEAD moved, so the stamp froze at whatever commit was checked out when the
# build directory was first created. Observed on a working tree:
#
#     $ git rev-parse --short=7 HEAD     -> 803733d
#     $ ./build/sat-tracker --version    -> SAT ... 0.1.0+d563657
#     $ run.json .provenance.build       -> d563657          (two commits stale)
#
# CMakeLists.txt's own justification for the field is that "every run.json and
# centroid.csv header carries the commit it was produced by... without it a
# benchmark number cannot be traced to code". A stale hash is worse than no
# hash: it traces the number to the WRONG code, confidently.
#
# ---------------------------------------------------------------------------
# WHY A GENERATED HEADER AND NOT CMAKE_CONFIGURE_DEPENDS
# ---------------------------------------------------------------------------
# Adding .git/HEAD and .git/index to CMAKE_CONFIGURE_DEPENDS would also fix the
# staleness, and it was the obvious first idea. It re-runs the whole configure
# step — the dependency search, the INV-1 link-closure walk, the whole module
# graph — every time anything is committed or even staged, which on this
# project is about ten seconds of wall time per `git add`.
#
# A generated header costs one `git rev-parse` per build and recompiles exactly
# the translation units that include it. configure_file() writes the file only
# when its contents would change, so an unchanged HEAD produces no timestamp
# update and therefore no rebuild at all.
#
# ---------------------------------------------------------------------------
# THE -dirty SUFFIX
# ---------------------------------------------------------------------------
# A hash alone still mis-attributes in the most common case of all: a benchmark
# run from a tree with uncommitted edits. The number came from code that exists
# in no commit, and `0.1.0+803733d` claims otherwise. `0.1.0+803733d-dirty`
# says what actually happened, and is exactly the signal a reader needs to
# distrust a figure.
#
# Only tracked files count. Untracked build output, logs and scratch files are
# not part of the build and should not flag it.
#
# Inputs (passed with -D):
#   SAT_SOURCE_DIR   the repository root
#   SAT_OUT          the header file to write

if(NOT DEFINED SAT_SOURCE_DIR OR NOT DEFINED SAT_OUT)
    message(FATAL_ERROR "stamp_git.cmake needs -DSAT_SOURCE_DIR= and -DSAT_OUT=")
endif()

set(SAT_GIT_HASH "unknown")

find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=7 HEAD
        WORKING_DIRECTORY "${SAT_SOURCE_DIR}"
        OUTPUT_VARIABLE _hash
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _rc
    )
    if(_rc EQUAL 0 AND _hash)
        set(SAT_GIT_HASH "${_hash}")

        # `diff-index --quiet HEAD --` exits non-zero when a TRACKED file
        # differs from HEAD. It is the cheap form: it does not walk untracked
        # files the way `status --porcelain` does, which on this tree means it
        # ignores build/, logs/ and dist/ without needing an ignore list.
        #
        # `update-index --refresh` first, because diff-index compares stat data
        # and reports spurious differences for files whose timestamps changed
        # without their contents changing — which is every file a build just
        # touched. Its own exit status is deliberately discarded.
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" update-index -q --refresh
            WORKING_DIRECTORY "${SAT_SOURCE_DIR}"
            OUTPUT_QUIET ERROR_QUIET
        )
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" diff-index --quiet HEAD --
            WORKING_DIRECTORY "${SAT_SOURCE_DIR}"
            RESULT_VARIABLE _dirty
            OUTPUT_QUIET ERROR_QUIET
        )
        if(NOT _dirty EQUAL 0)
            set(SAT_GIT_HASH "${SAT_GIT_HASH}-dirty")
        endif()
    endif()
endif()

# configure_file rather than file(WRITE): it compares the rendered contents
# against the existing file and leaves the timestamp alone when they match, so
# a build with an unchanged HEAD triggers no recompilation.
configure_file("${SAT_SOURCE_DIR}/cmake/build_stamp.hpp.in" "${SAT_OUT}" @ONLY)
