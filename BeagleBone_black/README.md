# BeagleBone Black

## Build
To build the project, run `make` from the top project directory. This shall run Lint checks to verify source files comply with the set style guide. Then, .cpp files are compiled and linked to create a binary specified by Makefile variable TARGET (defaults to "hyped").

## New project files
Build script must be notified of all new files that become part of the project.

When you add a new .cpp file to the project, make sure you record it in file `src/Source.files` to ensure it is compiled into the project's binary.

When you add a new .cpp or .hpp file to the project, make sure you record it in file `src/Lint.files` to ensure it is style-checked by linter.

If you want to ignore a lint error for particular line in your source code, add `// NOLINT [rule]` at the end of the line, e.g. `// NOLINT [whitespace/line_length]`. [rule] part is optional, it selects which lint rule checking should be skipped. If no rule is provided, all lint rules will be skipped.

## Project Organisation
Project is organised in a modular fashion to decrease dependencies among software teams.
### Multiple main()s
Project can be built using any main file located in folder `src/`. To use particular main file for creating project's binary run `make MAIN=<your_main_file>`, defaults to "main.cpp".

Make sure the project never compiles .cpp files containing multiple definitions of function `main()`. All example main files should be located at the top level source directory `src/`.

### Namespaces
Namespaces in all source files should reflect the folder hierarchy of the files. Folder `src/` should be replaced by `hyped::` in the namespace hierarchy. For example, contents of file `src/i2c/channel.hpp` should live in namespace `hyped::i2c`.