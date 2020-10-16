# GL_final_project

#LED strip controller with infrared remote control

## Technologies
Project is created with:
* C11
* libopencm3
* OpenOCD

Requirements
************

- `OpenOCD <http://openocd.org>`_.
  
  | Stable version: ``openocd``
    (Not recommended as it is outdated and incompatible with our openocd config.
     If you prefer stable version, use ``openocd -f board/stm32f4discovery.cfg``
     instead of our ``openocd -f openocd_glstarterkit.cfg``)
  | Latest Git version: ``openocd-git`` (through AUR)
- `arm-none-eabi Toolchain <https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm>`_
  
  Official release verison:
     
     | `GCC <https://gcc.gnu.org/>`_: ``arm-none-eabi-gcc``
     | `Binutils <https://www.gnu.org/software/binutils/>`_: ``arm-none-eabi-binutils``
     | `GDB <https://www.gnu.org/software/gdb/>`_: ``arm-none-eabi-gdb``
     | `Newlib <https://sourceware.org/newlib/>`_: ``arm-none-eabi-newlib``
- `Doxygen <https://doxygen.nl>`_ and `GraphViz <https://graphviz.org/>`_ for building libopencm3 documentation
  
  ``doxygen`` and ``graphviz`` packages

Add user to plugdev group:
~~~~~~~~~~~~~~~~~~~~~~~~~
This step is required to allow working with debuggers OpenOCD supports as a user, without a need
for having root privileges.

OpenOCD package on Arch comes with udev rules file (``/usr/lib/udev/rules.d/60-openocd.rules``).
It gives access rights to users in plugdev group, which exists on Debian, but is not present
on Arch Linux. So we need to create the group and add our user to it:

.. code-block:: shell-session
   
   sudo groupadd -r -g 46 plugdev
   sudo useradd -G plugdev $USER

And log out (or reboot)

Install all packages:
~~~~~~~~~~~~~~~~~~~~~
.. code-block:: shell-session
   
   yay -S openocd-git
   sudo pacman -S arm-none-eabi-{gcc,binutils,gdb,newlib} doxygen graphviz

.. note::
   You need to either run ``sudo udevadm control --reload-rules`` and ``sudo udevadm trigger``
   or to reboot after installing OpenOCD for udev rules to start working

##Run
To run this project,use:
	make PROFILE=release target
	make PROFILE=release flash

##Modes:

button 1: running line
button 2: flicker 1
button 3: constant glow
button 4: flicker2

