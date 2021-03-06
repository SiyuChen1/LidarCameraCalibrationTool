Librviz Tutorial
================

Overview
--------

RViz is not just a visualizer application, it is also a library!  Much
of RViz's functionality can be accessed within your own application by
linking against librviz.so (or whatever your OS likes to call it).

This tutorial shows a very simple example of creating a 3D visualizer
widget (rviz::RenderPanel), programmatically creating a new Grid
display within it, then using Qt slider controls to adjust a couple of
the grid's properties.  The app is called "PointCloud2Display".

The source code for this tutorial is in the librviz_tutorial
package. You can check out the source directly or (if you use Ubuntu)
you can just apt-get install the pre-compiled Debian package like so::

    sudo apt-get install ros-hydro-visualization-tutorials

The running application looks like this:

.. image:: PointCloud2Display.png

The Code
--------

The code for PointCloud2Display is in these files: 
:codedir:`src/main.cpp`,
:codedir:`src/PointCloud2Display.h`, and
:codedir:`src/PointCloud2Display.cpp`.

main.cpp
^^^^^^^^

The full text of main.cpp is here: :codedir:`src/main.cpp`

.. tutorial-formatter:: ../main.cpp

PointCloud2Display.h
^^^^^^^

The full text of PointCloud2Display.h is here: :codedir:`src/PointCloud2Display.h`

.. tutorial-formatter:: ../PointCloud2Display.h

PointCloud2Display.cpp
^^^^^^^^^

The full text of PointCloud2Display.cpp is here: :codedir:`src/PointCloud2Display.cpp`

.. tutorial-formatter:: ../PointCloud2Display.cpp

Building
--------

The full text of CMakeLists.txt is here: :codedir:`CMakeLists.txt`

.. tutorial-formatter:: ../../CMakeLists.txt

Running
-------

Just type::

    rosrun librviz_tutorial PointCloud2Display
