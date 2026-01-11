# Instructions

Set up a build conda environment and build the package(s).

```bash
conda create -n roboplan-build conda-build conda-forge-pinning
conda activate roboplan-build
conda build recipe/
```

Test the built package(s) by setting up a new testing conda environment.

```bash
conda create -n roboplan-test -c ~/miniconda3/envs/roboplan-build/conda-bld/ roboplan-python
conda activate roboplan-test
python -c "import roboplan"
```