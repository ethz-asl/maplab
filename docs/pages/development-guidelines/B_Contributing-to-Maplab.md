## Contributing to maplab

We would like to ask you to stick to the following procedure when using the maplab repository:

- Use the google-c++ code [style](https://google.github.io/styleguide/cppguide.html).
- Follow the pull-request and review work-flow.
- Never push directly to master.
- Never merge branches that did not pass the build-server test-build.
- Never merge branches with failing builds.
- Prefer pull-requests which are focused on a particular feature/fix.
- Prefer pull-requests that have a change-list of < 500 lines.
- Write unit-tests for your code using gtest.
- Follow the [[Verbosity Policy]]

- Write at least one high level comment per method and class on the purpose unless the method name is entirely self explanatory.
- Avoid comments that don't contain additional information beyond the obvious.

After you cloned you have to run the following script to setup an auto-formatter and static-code analysis tool:
```bash
cd maplab
./tools/linter/init-git-hooks.py
```
