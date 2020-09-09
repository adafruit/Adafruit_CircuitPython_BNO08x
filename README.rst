Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bno08x/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/bno08x/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/actions
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

Helper library for the Hillcrest Laboratories BNO08x IMUs


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-bno08x/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-bno08x

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-bno08x

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-bno08x

Usage Example
=============

.. code-block:: python3

    import board
    import busio
    import adafruit_bno08x

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno08x.BNO08X(i2c)

    while True:
        quat = bno.rotation_vector
        print("Rotation Vector Quaternion:")
        print("I: %0.3f J: %0.3f K: %0.3f Accuracy: %0.3f"%(quat.i, quat.j, quat.k, quat.accuracy))

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
