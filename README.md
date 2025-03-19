# ODrive Driver
This repositry contains low level driver code for the ODrive Motor Controllers in C++ with Python bindings.

# Usage
This repository is intended to be used as an external target for Bazel builds.
Add the following to your `MODULE.bazel` file:

```python
bazel_dep(name = "odrive-bazel")
archive_override(
    module_name = "odrive-bazel",
    strip_prefix = "odrive-bazel-main",
    urls = ["https://github.com/jeh15/odrive-bazel/archive/refs/heads/main.zip"],
)
```
