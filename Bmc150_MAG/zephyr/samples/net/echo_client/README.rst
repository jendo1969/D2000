Echo Client
###########

Overview
********

The echo-client sample application for Zephyr implements a UDP/TCP client
that will send IPv4 or IPv6 packets, wait for the data to be sent back,
and then verify it matches the data that was sent.

The source code for this sample application can be found at:
:file:`samples/net/echo_client`.

Building and Running
********************

There are multiple ways to use this application. One of the most common
usage scenario is to run echo-client application inside QEMU. This is
described in :ref:`networking with QEMU <networking_with_qemu>`.

There are configuration files for different boards and setups in the
echo-client directory:

- :file:`prj_arduino_101_cc2520.conf`
  Use this for Arduino 101 with external IEEE 802.15.4 cc2520 board.

- :file:`prj_arduino_101.conf`
  Use this for Arduino 101 with external enc28j60 ethernet board.

- :file:`prj_bt.conf`
  Use this for Bluetooth IPSP connectivity.

- :file:`prj_cc2520.conf`
  Use this for devices that have support for IEEE 802.15.4 cc2520 chip.

- :file:`prj_frdm_k64f_cc2520.conf`
  Use this for FRDM-K64F board with external IEEE 802.15.4 cc2520 board.

- :file:`prj_frdm_k64f.conf`
  Use this for FRDM-K64F board with built-in ethernet.

- :file:`prj_frdm_k64f_mcr20a.conf`
  Use this for FRDM-K64F board with IEEE 802.15.4 mcr20a board.

- :file:`prj_qemu_802154.conf`
  Use this when simulating IEEE 802.15.4 network using two QEMU's that
  are connected together.

- :file:`prj_qemu_cortex_m3.conf`
  Use this for ARM QEMU.

- :file:`prj_qemu_x86.conf`
  Use this for x86 QEMU.

- :file:`prj_sam_e70_xplained.conf`
  Use this for Atmel SMART SAM E70 Xplained board with ethernet.

Build echo-client sample application like this:

.. code-block:: console

    $ cd $ZEPHYR_BASE/samples/net/echo_client
    $ make pristine && make CONF_FILE=<your desired conf file> \
      BOARD=<board to use>

Make can select the default configuration file based on the BOARD you've
specified automatically so you might not always need to mention it.

Running echo-server Linux Host
==============================

There is one useful testing scenario that can be used with Linux host.
Here echo-client is run in QEMU and echo-server is run in Linux host.

Run 'loop_socat' and 'loop-slip-tap' scripts from net-tools in Linux host.

.. code-block:: console

    $ ./loop_socat.sh

In another window:

.. code-block:: console

    $ sudo ./loop-slip-tap.sh

In third window:

.. code-block:: console

    $ sudo ./echo-server -i tap0

Run echo-client application in QEMU:

.. code-block:: console

    $ cd $ZEPHYR_BASE/samples/net/echo-client
    $ make pristine && make qemu
