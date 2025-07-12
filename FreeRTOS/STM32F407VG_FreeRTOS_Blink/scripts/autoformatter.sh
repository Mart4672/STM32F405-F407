#!/usr/bin/env bash

# RELATIVE_SCRIPT_DIR=$(dirname "$0")
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=${SCRIPT_DIR}/..

# Read the clang documentation for more information on Clang-Format Style Options
# https://clang.llvm.org/docs/ClangFormatStyleOptions.html

# Command to generate a .clang-format file:
# clang-format -style=llvm -dump-config > .clang-format
# currently using clang-format version 14.0.0-1ubuntu1.1

# Formatting can be turned off for sections of code using the following:
# // clang-format off
#     void    unformatted_code  ;
# // clang-format on

# run autoformatter from the project directory
pushd ${PROJECT_DIR}
    # use the clang-format package to autoformat files in this directory
    git ls-files | grep -iE ".*\.(cpp|c|hpp|h)" | xargs clang-format -i
popd
