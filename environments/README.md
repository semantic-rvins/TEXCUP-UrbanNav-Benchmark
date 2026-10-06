# Reproducible environments

The explicit Conda files capture the Linux x86-64 package sets used by the
completed runs. Create only the environment needed, using the commands in
[run setup](../RUN_INSTRUCTIONS.md).

| File | Consumers |
|---|---|
| `benchmark_ros.explicit.txt` | GVINS, VINS-Fusion, GICI image conversion |
| `benchmark_icgvins.explicit.txt` | IC-GVINS, with matching TBB runtime/headers |
| `benchmark_gici.explicit.txt` | Standalone GICI-RTK and GICI-RRR |
| `statistics-requirements.txt` | Python3.12 scoring without ROS or native estimators |

The Conda lists contain package URLs, not an installed environment prefix.
Exact package availability is required to recreate those locks. Use a fresh
shell when switching ROS workspaces. Raw bags and data are not environment
contents; prepare them using each method's guide.
