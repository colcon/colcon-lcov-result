colcon-lcov-result
==================

An extension for `colcon-core <https://github.com/colcon/colcon-core>`_ to provide aggregate
coverage results using `LCOV <http://ltp.sourceforge.net/coverage/lcov.php>`_.

LCOV is a graphical front-end for GCC's coverage testing tool
`gcov <https://gcc.gnu.org/onlinedocs/gcc/Gcov.html>`_, producing the following
coverage metrics:

- Statement coverage
- Function coverage
- Branch coverage

For more information, see `this paper
<http://ltp.sourceforge.net/documentation/technical_papers/gcov-ols2003.pdf>`_
and `this Wikipedia page <https://en.wikipedia.org/wiki/Code_coverage>`_.


Usage
=====
#. Build your packages with coverage flags, using ``colcon``:

   .. code-block:: shell

     $ colcon build \
           --symlink-install \
           --cmake-args \
               -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' \
               -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'

   * See also `colcon-mixin <https://github.com/colcon/colcon-mixin>`_ and 
     `colcon-mixin-repository <https://github.com/colcon/colcon-mixin-repository/blob/master/coverage.mixin>`_
     for a short-hand command (``--mixin coverage-gcc``)
  
#. Create a baseline for zero coverage:

   .. code-block:: shell

     $ colcon lcov-result --initial
  
   * This step is optional, but will help reveal any files that are untouched by
     tests

#. Run tests:

   .. code-block:: shell

     $ colcon test

#. Gather the ``lcov`` results:

   .. code-block:: shell

     $ colcon lcov-result
     Reading tracefile /home/user/workspace/my_cool_ws/lcov/total_coverage.info
     Summary coverage rate:
       lines......: 78.6% (44 of 56 lines)
       functions..: 94.4% (34 of 36 functions)
       branches...: 37.0% (34 of 92 branches)

#. Browse the coverage report by opening ``lcov/index.html`` in a browser

#. Zero the coverage counters and re-run tests:

   .. code-block:: shell

     $ colcon lcov-result --zero-counters
     $ colcon lcov-result --initial
     $ colcon test
     $ colcon lcov-result
     Reading tracefile /home/user/workspace/my_cool_ws/lcov/total_coverage.info
     Summary coverage rate:
       lines......: 78.6% (44 of 56 lines)
       functions..: 94.4% (34 of 36 functions)
       branches...: 37.0% (34 of 92 branches)


Tips and Tricks
===============

* When running locally, use the ``--packages-select`` option to generate
  coverage information for relevant packages
  
  * This will also suppress warnings for packages that were either not built
    with coverage flags or for which tests did not run

* The ``--verbose`` flag can be used to print the coverage summary of each
  individual package as the results are analyzed


Contributing
============

For non-trivial contributions, it is recommended to first create an issue to discuss
your ideas.

The following is the recommended workflow for contributing:

#. Install ``colcon`` and extensions in a virtual environment:

   .. code-block:: shell

     $ cd <workspace>
     $ python3 -m venv colcon-env
     $ source colcon-env/bin/activate
     $ pip3 install colcon-common-extensions

#. Install ``colcon-lcov-result`` in editable mode:

   .. code-block:: shell

     $ cd <workspace>
     $ python3 -m venv colcon-env
     $ source colcon-env/bin/activate
     $ cd path/to/colcon-lcov-result
     $ pip3 install -e .

#. As long as you are in the virtual environment, make changes to ``colcon-lcov-result``
   run ``colcon lcov-result``, and see the effect of the changes

#. Commit changes and submit a PR:

   * See `The seven rules of a great Git commit message`_

.. _The seven rules of a great Git commit message: https://chris.beams.io/posts/git-commit/#seven-rules


Troubleshooting
===============

* The following warning when running ``colcon lcov-result --initial`` implies
  that the package was not built with the correct flags:

  .. code-block:: shell
  
     --- stderr: my_pkg                                                        
     geninfo: WARNING: no .gcno files found in /home/user/workspace/build/my_pkg - skipping!
     ---

  * The package will not show up in the final results. Use ``--packages-skip`` to suppress
    the warning

* The following warning when running ``colcon lcov-result`` implies that no tests
  ran for that package
  
  .. code-block:: shell

     [0.576s] ERROR:colcon.colcon_lcov_result.task.lcov:lcov:
     ERROR: no valid records found in tracefile /home/user/workspace/build/my_pkg/coverage.info
     --- stderr: my_pkg
     geninfo: WARNING: no .gcda files found in /home/user/workspace/build/my_pkg - skipping!
     ---

  * The package will show up in the final results with 0% coverage. Use ``--packages-skip``
    to suppress these packages from the total


Known Issues
============

#. The final step of aggregating all the result files can be slow depending
   on the number of packages that were analyzed
