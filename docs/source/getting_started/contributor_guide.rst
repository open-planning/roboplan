Contributor Guide
=================

Thank you for considering a contribution to RoboPlan!

Please make sure to follow the :doc:`design philosophy </design>` and :doc:`Installation </getting_started/installation>` pages.

If you're not sure what to work on, take a look at any of the `open issues <https://github.com/open-planning/roboplan/issues>`_ or the :doc:`Project Ideas </getting_started/project_ideas>` pages.


Creating Python Bindings
------------------------

We use `nanobind <https://github.com/wjakob/nanobind>`_ to generate Python bindings.
As described in the :doc:`Design Philosophy </design>` page, you should bias towards implementing functionality in C++ and creating Python bindings.
There are exceptions; for example, if you're making Python-specific utilities for examples, visualization, etc.

The Python bindings source code can be found in the ``bindings`` subfolder of the repository.
When you build the project, the bindings should also be built automatically.

``nanobind`` has the ability to `generate stub files <https://nanobind.readthedocs.io/en/latest/typing.html#stub-generation>`_ for type checking and IDE integration.
Depending on how you build the project, these stub files may be symbolic links or use a different version of ``nanobind`` than what we standardize on.
The best way of making sure the stubs are up to date is by using Pixi:

::

    pixi run -e ci build_all

This uses the ``ci`` environment in our Pixi project to ensure that the build is done via copy, not symbolic link mechanism.

... but don't worry!
There is a helpful CI check that will tell you if the stub files don't agree.


Running Tests and Generating Documentation
------------------------------------------

When working on your contributions, make sure the unit tests pass and the documentation looks good.
Of course, you should add your own tests and documentation depending on what you are working on!
When adding unit tests, ensure you test both the C++ and Python sides (Coding Assistants are great at porting tests over to other languages).

Running the tests depends on the installation workflow you used.
Testing instructions are available in the :doc:`Installation </getting_started/installation>` page.

If you are making changes that you think could affect the examples, we also recommend manually running the relevant examples to check that they still work.
(Or even better, if you have ideas for automatically testing all the examples, that would be a great contribution!)

To generate the documentation on your end, ``cd`` to the ``docs`` subfolder of the repository.

First, install the requirements (we recommend using a virtual environment).

::

    pip3 install -r python_docs_requirements.txt

Then, build the documentation.

::

    rm -rf build/
    make html

You can view the generated documentation in your browser.

::

    open build/html/index.html

.. note::

   The C++ and Python API docs should be automatically generated for you when you build the docs!


When you submit a pull request, you can also access the built documentation from your branch from the `ReadTheDocs <https://about.readthedocs.com/>`_ CI job.


On Coding Assistant Usage
-------------------------

Coding assistants such as Claude Code, GitHub Copilot, etc. are powerful tools that contributors are welcome to use!
However, be advised that *you are ultimately responsible for your code*.
Please make sure you aren't just "vibe coding" changes without reviewing your work first.

The maintainers will close contributions that are obviously AI generated without verification.
