echo "Running command \"clang-tidy -config-file=.clang-tidy -p build/compile_commands.json [files]\""
echo "Files:"

find \
    main \
    \( -iname *.h -o -iname *.cpp \) \
    -exec echo " " {} \; \
    -exec clang-tidy-15 -config-file=.clang-tidy -p build/compile_commands.json {} +
