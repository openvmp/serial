^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remote_serial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-05-06)
------------------

1.0.0 (2023-05-01)
------------------
* Renamed the package to better align with REP-0144 guidelines
* Fixed missing class member initialization for the remote interface. Minor performance improvements.
* Compiles on foxy
* Refactored the boundary between the serial protocol implementation class and the serial port class as deemed necessary by an implementation of UART on a microcontroller.
* Flow control refactoring. Reduced overhead from debug logging.
* Refactored DDS path for all resources
* Work around a timing issue in the test cases
* Interface is refactored to support a factory and to coexist with other OpenVMP modules
* Made it work as a dependency for other packages. Made it work with binary data.
* updated the documentation to reflect the recent internal terminology change
* added the first test case and made it work
* First version that compiles
* added the openvmp banner
* added README.md
* Initial revision
* Contributors: Roman Kuzmenko
