## Generating Documentation

Make sure you `cd` to this folder.

First, install the requirements (recommend using a virtual environment).

```bash
pip install -r python_docs_requirements.txt
```

Then, build the documentation.

```bash
rm -rf build/
make html
```
