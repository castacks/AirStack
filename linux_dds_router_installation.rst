.. include:: ../../../exports/alias.include
.. include:: ../../../exports/roles.include

.. _developer_manual_installation_sources_linux:

###############################
Linux installation from sources
###############################

The instructions for installing the |ddsrouter| application from sources and its required
dependencies are provided in this page.
It is organized as follows:

Dependencies installation
=========================

|ddsrouter| depends on *eProsima Fast DDS* library and certain Debian packages.
This section describes the instructions for installing |ddsrouter| dependencies and requirements in a Linux
environment from sources.
The following packages will be installed:

- ``foonathan_memory_vendor``, an STL compatible C++ memory allocation library.
- ``fastcdr``, a C++ library that serializes according to the standard CDR serialization mechanism.
- ``fastdds``, the core library of eProsima Fast DDS library.
- ``cmake_utils``, an eProsima utils library for CMake.
- ``cpp_utils``, an eProsima utils library for C++.

First of all, the :ref:`Requirements <requirements>` and :ref:`Dependencies <dependencies>` detailed below need to be
met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation>` or the
:ref:`CMake <cmake_installation>` installation instructions.

.. _requirements:

Requirements
------------

The installation of |ddsrouter| in a Linux environment from sources requires the following tools to be
installed in the system:

* :ref:`cmake_gcc_pip_wget_git_sl`
* :ref:`colcon_install` [optional]
* :ref:`gtest_sl` [for test only]
* :ref:`py_yaml` [for YAML Validator only]
* :ref:`json_schema` [for YAML Validator only]


.. _cmake_gcc_pip_wget_git_sl:

CMake, g++, pip, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install |ddsrouter| and its dependencies from command line.
Install CMake_, `g++ <https://gcc.gnu.org/>`_, pip_, wget_ and git_ using the package manager of the appropriate
Linux distribution. For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install cmake g++ pip wget git


.. _colcon_install:

Colcon
^^^^^^

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

.. code-block:: bash

    pip3 install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.


.. _gtest_sl:

Gtest
^^^^^

Gtest_ is a unit testing library for C++.
By default, |ddsrouter| does not compile tests.
It is possible to activate them with the opportune
`CMake options <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-options>`_
when calling colcon_ or CMake_.
For more details, please refer to the :ref:`cmake_options` section.
For a detailed description of the Gtest_ installation process, please refer to the
`Gtest Installation Guide <https://github.com/google/googletest>`_.

It is also possible to clone the Gtest_ Github repository into the |ddsrouter| workspace and compile it with colcon_
as a dependency package.
Use the following command to download the code:

.. code-block:: bash

    git clone --branch release-1.12.0 https://github.com/google/googletest src/googletest-distribution


.. _py_yaml:

PyYAML
^^^^^^

`PyYAML <https://pyyaml.org/>`_ is a YAML parser and emitter for Python.

It is used by the DDS-Router :ref:`yaml_validator` for loading the content of configuration files.

Install ``pyyaml`` by executing the following command:

.. code-block:: bash

    pip3 install -U pyyaml


.. _json_schema:

jsonschema
^^^^^^^^^^

`jsonschema <https://python-jsonschema.readthedocs.io/>`_ is an implementation of the JSON Schema specification for
Python.

It is used by the DDS-Router :ref:`yaml_validator` for performing validation of configuration files against a given
JSON schema.

Install ``jsonschema`` by executing the following command:

.. code-block:: bash

    pip3 install -U jsonschema

.. _dependencies:

Dependencies
------------

|ddsrouter| has the following dependencies, when installed from sources in a Linux environment:

* :ref:`asiotinyxml2_sl`
* :ref:`openssl_sl`
* :ref:`yaml_cpp`
* :ref:`eprosima_dependencies`

.. _asiotinyxml2_sl:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install libasio-dev libtinyxml2-dev

.. _openssl_sl:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install OpenSSL_ using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt install libssl-dev

.. _yaml_cpp:

yaml-cpp
^^^^^^^^

yaml-cpp is a YAML parser and emitter in C++ matching the YAML 1.2 spec, and is used by *DDS Router* application to
parse the provided configuration files.
Install yaml-cpp using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt install libyaml-cpp-dev

.. _eprosima_dependencies:

eProsima dependencies
^^^^^^^^^^^^^^^^^^^^^

If it already exists in the system an installation of *Fast DDS* library with version greater than `2.4.0`, just source
this library when building the |ddsrouter| application by using the command:

.. code-block:: bash

    source <fastdds-installation-path>/install/setup.bash

In other case, just download *Fast DDS* project from sources and build it together with |ddsrouter| using colcon
as it is explained in section :ref:`colcon_installation`.


.. _colcon_installation:

Colcon installation
===================

#.  Create a :code:`DDS-Router` directory and download the :code:`.repos` file that will be used to install
    |ddsrouter| and its dependencies:

    .. code-block:: bash

        mkdir -p ~/DDS-Router/src
        cd ~/DDS-Router
        wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
        vcs import src < ddsrouter.repos

    .. note::

        In case there is already a *Fast DDS* installation in the system it is not required to download and build
        every dependency in the :code:`.repos` file.
        It is just needed to download and build the |ddsrouter| project having sourced its dependencies.
        Refer to section :ref:`eprosima_dependencies` in order to check how to source *Fast DDS* library.

#.  Build the packages:

    .. code-block:: bash

        colcon build

.. note::

    Being based on CMake_, it is possible to pass the CMake configuration options to the :code:`colcon build`
    command. For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.


.. _cmake_installation:

CMake installation
==================

This section explains how to compile |ddsrouter| with CMake_, either
:ref:`locally <local_installation_sl>` or :ref:`globally <global_installation_sl>`.

.. _local_installation_sl:

Local installation
------------------

#.  Create a :code:`DDS-Router` directory where to download and build |ddsrouter| and its dependencies:

    .. code-block:: bash

        mkdir -p ~/DDS-Router/src
        mkdir -p ~/DDS-Router/build
        cd ~/DDS-Router
        wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
        vcs import src < ddsrouter.repos

#.  Compile all dependencies using CMake_.

    * `Foonathan memory <https://github.com/foonathan/memory>`_

      .. code-block:: bash

          cd ~/DDS-Router
          mkdir build/foonathan_memory_vendor
          cd build/foonathan_memory_vendor
          cmake ~/DDS-Router/src/foonathan_memory_vendor -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DBUILD_SHARED_LIBS=ON
          cmake --build . --target install

    * `Fast CDR <https://github.com/eProsima/Fast-CDR>`_

      .. code-block:: bash

          cd ~/DDS-Router
          mkdir build/fastcdr
          cd build/fastcdr
          cmake ~/DDS-Router/src/fastcdr -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install
          cmake --build . --target install

    * `Fast DDS <https://github.com/eProsima/Fast-DDS>`_

      .. code-block:: bash

          cd ~/DDS-Router
          mkdir build/fastdds
          cd build/fastdds
          cmake ~/DDS-Router/src/fastdds -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install

    * `Dev Utils <https://github.com/eProsima/dev-utils>`_

      .. code-block:: bash

          # CMake Utils
          cd ~/DDS-Router
          mkdir build/cmake_utils
          cd build/cmake_utils
          cmake ~/DDS-Router/src/dev-utils/cmake_utils -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install

          # C++ Utils
          cd ~/DDS-Router
          mkdir build/cpp_utils
          cd build/cpp_utils
          cmake ~/DDS-Router/src/dev-utils/cpp_utils -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install

    * `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_

      .. code-block:: bash

          # ddspipe_core
          cd ~/DDS-Router
          mkdir build/ddspipe_core
          cd build/ddspipe_core
          cmake ~/DDS-Router/src/ddspipe/ddspipe_core -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install

          # ddspipe_participants
          cd ~/DDS-Router
          mkdir build/ddspipe_participants
          cd build/ddspipe_participants
          cmake ~/DDS-Router/src/ddspipe/ddspipe_participants -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install

          # ddspipe_yaml
          cd ~/DDS-Router
          mkdir build/ddspipe_yaml
          cd build/ddspipe_yaml
          cmake ~/DDS-Router/src/ddspipe/ddspipe_yaml -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
          cmake --build . --target install


#.  Once all dependencies are installed, install |ddsrouter|:

    .. code-block:: bash

        # ddsrouter_core
        cd ~/DDS-Router
        mkdir build/ddsrouter_core
        cd build/ddsrouter_core
        cmake ~/DDS-Router/src/ddsrouter/ddsrouter_core -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
        cmake --build . --target install

        # ddsrouter_yaml
        cd ~/DDS-Router
        mkdir build/ddsrouter_yaml
        cd build/ddsrouter_yaml
        cmake ~/DDS-Router/src/ddsrouter/ddsrouter_yaml -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
        cmake --build . --target install

        # ddsrouter_tool
        cd ~/DDS-Router
        mkdir build/ddsrouter_tool
        cd build/ddsrouter_tool
        cmake ~/DDS-Router/src/ddsrouter/tools/ddsrouter_tool -DCMAKE_INSTALL_PREFIX=~/DDS-Router/install -DCMAKE_PREFIX_PATH=~/DDS-Router/install
        cmake --build . --target install

.. note::

    By default, |ddsrouter| does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_
    and building with CMake option ``-DBUILD_TESTS=ON``.


.. _global_installation_sl:

Global installation
-------------------

To install |ddsrouter| system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR`, :code:`Fast-DDS`, and
:code:`DDS-Router`, and change the first in the configuration step of :code:`foonathan_memory_vendor` to the
following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON

.. _run_app_colcon_sl:

Run an application
==================

To run the |ddsrouter| application, source the *Fast DDS* library
and execute the executable file that has been installed in :code:`<install-path>/ddsrouter_tool/bin/ddsrouter`:

.. code-block:: bash

    # If built has been done using colcon, all projects could be sourced as follows
    source install/setup.bash
    ./<install-path>/ddsrouter_tool/bin/ddsrouter

Be sure that this executable has execute permissions.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip: https://pypi.org/project/pip/
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
