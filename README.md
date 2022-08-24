environments and algorithms for exploration

Implemented an exploration simulation framework with modularity in:
 - Sensors: Neighborhood, 2D-Lidar
 - Exploration Algorithms: Frontier-Based Exploration
 - Mapping: Log-Odds Mapping
 - Motion Planners: Weighted A*

Can explore any environment that is given as a costmap
Can easily plug in play other planners / mappers
Frontier-based exploration implemented with a few different behavioral modes, each one explores a bit differently.

Includes integration with online mapping.
Includes benchmarking framework / benchmarks

# Install

## Option 1.
```shell
pip3 install numpy==1.22.3
python3 setup.py bdist
pip3 install dist/diff_info_gathering-0.1.0-py3-none-any.whl --force-reinstall
```

## Option 2.
```shell
pip3 install . --force-reinstall
```
