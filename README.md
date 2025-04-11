# AIROC&trade; CYW20829: Bluetooth&reg; Low Energy Isochronous Central

The following two examples, when executed in parallel on two separate boards, demonstrate the use of Bluetooth&reg; Low Energy isochronous channels with Infineon AIROC&trade; CYW20829 Bluetooth&reg; devices.

1. **LE Isochronous peripheral application:** This code example is designed to connect to an isochronous channel central application. When powered up and after pressing the USER BTN1 button, it starts advertising for the central to connect to it and establish an isochronous channel connection.

2. **LE Isochronous central application:** This complementary application (this document) implements the Central role. When powered up and after pressing the USER BTN1 button, it scans for the isochronous peripheral advertisements and connects to the peripheral when found and establishes an isochronous channel connection.

These code examples working together demonstrate the implementation of sending generic data over isochronous channels. The central application can also support two Connected Isochronous Streams (CIS), i.e., two isochronous connections to two separate peripheral devices.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-btstack-freertos-le-isoc-central)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzg0NTMiLCJTcGVjIE51bWJlciI6IjAwMi0zODQ1MyIsIkRvYyBUaXRsZSI6IkFJUk9DJnRyYWRlOyBDWVcyMDgyOTogQmx1ZXRvb3RoJnJlZzsgTG93IEVuZXJneSBJc29jaHJvbm91cyBDZW50cmFsIiwicmlkIjoiamd1aSIsIkRvYyB2ZXJzaW9uIjoiMS4xLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiSUNXIiwiRG9jIEZhbWlseSI6IkJUQUJMRSJ9)



## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.2 or later (tested with v3.2)
- Board support package (BSP) minimum required version for CYW920829M2EVK-02: v1.0.2.37335
- Programming language: C
- Associated parts: All AIROC&trade; [CYW20829](https://www.infineon.com/cms/en/product/promopages/airoc20829/) Bluetooth&reg; LE SoCs


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler  v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.13 (`ARM`)
- IAR C/C++ Compiler v8.42.2 (`IAR`)


## Supported kits (make variable 'TARGET')

- AIROC&trade; [CYW920829M2EVK-02](https://www.infineon.com/cms/en/product/promopages/airoc20829/) (`CYW920829M2EVK-02`)


## Hardware setup

This example uses the kit's default configuration. See the respective kit guide to ensure that the kit is configured correctly.


## LED indications

### Central - LED1

LED state | Operation
----------|----------
Fast Blink (2 Hz) | High Duty Scan (default duration 30 s)
Slow Blink (1 Hz) | Low Duty Scan (default duration 30 s)
Glows | LE link is up
OFF | Link down

<br>

### Central - LED2

LED state | Operation
----------|----------
Blinks |  Trying to establish ISOC
Glows | ISOC channel is up. The LED state is toggled when the central receives data from the peripheral.

<br>

### Peripheral - LED1

LED state | Operation
----------|----------
Blinks slow (1 Hz) | Pairing (Undirected Advertising - by default continues to advertise until paired)
Glows | LE link up
OFF | Link down

<br>

### Peripheral - LED2

LED state | Operation
----------|----------
Glows | ISOC channel up. The LED state is toggled when the peripheral receives data from the central.


## Button functions

### Central BTN1

Link status | Button function
------------|---------------
Not Connected | 1. Press and release to start scanning for a LE ISOC peripheral to connect to. If the number of connections equals to the MAX_CONNECTED_DEVICES defined in the makefile, an isochronous channel connection will be initiated. <br> 2. If the number of connections is less than the MAX_CONNECTED_DEVICES, press and release again to start scanning for the 2nd LE ISOC peripheral to connect to. Isochronous channel connections will be initiated to both peripherals after the LE connection to the 2nd device is successful.
ISOC channel(s) is up | Sends ISOC data to peripheral for each button transition. # of packets sent determined by ISOC_MAX_BURST_COUNT defined in the isoc_central.c source file.


### Peripheral BTN1

LED state | Operation
----------|----------
LED1 and LED2 OFF | Starts advertising.
LED1 blinks slow (undirected advertising for pairing)| Exit advertising mode.
LED2 glows, ISOC channel is up | Sends ISOC data to central for each button transition. # of packets sent determined by ISOC_MAX_BURST_COUNT defined in the isoc_peripheral.c source file.

### Reset button

This button resets the system. The pairing information is erased; the device automatically entering pairing state.
When reset buttons are pressed at both boards, the boards pair and connect over LE.


## Software setup

Use BTSpy to connect to the COM port of the target board at 3 Mbps.


## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade;</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; provides the Project Creator as both a GUI tool and the command line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The "project-creator-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

The following example clones the "[LE ISOC Central](https://github.com/Infineon/mtb-example-btstack-freertos-le-isoc-central)" application with the desired name "le-isoc-central" configured for the *CYW920829M2EVK-02* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CYW920829M2EVK-02 --app-id mtb-example-btstack-freertos-le-isoc-central --user-app-name le-isoc-central --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).


To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can invoke the Library Manager GUI tool from the terminal using `make library-manager` command or use the Library Manager CLI tool "library-manager-cli" to change the BSP.

The "library-manager-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--add-bsp-name` | Name of the BSP that should be added to the application | Required
`--set-active-bsp` | Name of the BSP that should be as active BSP for the application | Required
`--add-bsp-version`| Specify the version of the BSP that should be added to the application if you do not wish to use the latest from manifest | Optional
`--add-bsp-location`| Specify the location of the BSP (local/shared) if you prefer to add the BSP in a shared path | Optional

<br>

The following example adds the CYW920829M2EVK-02 BSP to the already created application and makes it the active BSP for the app:

   ```
   ~/ModusToolbox/tools_{version}/library-manager/library-manager-cli --project "C:/mtb_projects/le-isoc-central" --add-bsp-name CYW920829M2EVK-02 --add-bsp-version "latest-v4.X" --add-bsp-location "local"

   ~/ModusToolbox/tools_{version}/library-manager/library-manager-cli --project "C:/mtb_projects/le-isoc-central" --set-active-bsp APP_CYW920829M2EVK-02
   ```

</details>

<details><summary><b>In third-party IDEs</b></summary>

Use one of the following options:

- **Use the standalone [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool:**

   1. Launch Project Creator from the Windows Start menu or from *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/project-creator.exe*.

   2. In the initial **Choose Board Support Package** screen, select the BSP, and click **Next**.

   3. In the **Select Application** screen, select the appropriate IDE from the **Target IDE** drop-down menu.

   4. Click **Create** and follow the instructions printed in the bottom pane to import or open the exported project in the respective IDE.

<br>

- **Use command-line interface (CLI):**

   1. Follow the instructions from the **In command-line interface (CLI)** section to create the application.

   2. Export the application to a supported IDE using the `make <ide>` command.

   3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

## Operation

### Single Connected Isochronous Stream (CIS) configuration (default)

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 3,000,000 baud.

3. Program the LE ISOC central and LE ISOC peripheral code examples onto two separate CYW920829M2EVK-02 boards using one of the following options:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade;</b></summary>

      a. Select the application project in Project Explorer.

      b. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. Reset both central and peripheral boards.

   a. Press **BTN1** on the central to start scanning.

   b. Press **BTN1** on the peripheral to start advertising.

   c. An LE connection should automatically be established. Verify that LED1 on both boards are glowing.

   d. After the LE connection is up, the central will automatically establish an isochronous channel.

5. Press **BTN1** at the *peripheral* side to send data to the central.

   LED2 glowing indicates that ISOC is established; therefore, the data is sent via ISOC. When the central receives the data, it toggles the LED2 state.

6. Press **BTN1** at the *central* side to send data to the peripheral.

   LED2 glowing indicates that ISOC is established; therefore, the data is sent via ISOC. When the peripheral receives the data, it toggles the LED2 state.

   Isochronous channel statistics are printed every 5s. In an ideal environment, the isoc_tx_count on the local side should match the isoc_rx_count on the peer side.

### MAX_CONNECTED_DEVICES compiler option (CIS configuration parameter)

This parameter defines the number of peripherals and CIS channels to support (default is '1', maximum supported in this version is '2'). Do the following to exercise the two CIS configurations.

1. In the application Makefile, set `MAX_CONNECTED_DEVICES?=2`.

2. Program the *LE ISOC central* code example onto a CYW920829M2EVK-02 board.

3. Program the *LE ISOC peripheral* code example onto two additional CYW920829M2EVK-02 boards. Be sure to update the PERIPHERAL_ID in the le-isoc-peripheral application makefile to 1 or 2 to differentiate the Bluetooth&reg; device addresses between the two peripherals.

4. Power up both central and one of the peripheral boards.

   a. Press **BTN1** on the central to start scanning.

   b. Press **BTN1** on one of the peripherals to start advertising.

   c. An LE connection should be established automatically. Verify that LED1 on both boards are glowing.

5. Pair a second device.

   a. Press **BTN1** on the central to start scanning for the 2nd device.

   b. Press **BTN1** on the 2nd peripheral to start advertising.

   c. An LE connection should be established automatically. Verify that LED1 on the second device side is glowing, indicating that the LE connection is automatically established.

   d. After the LE connections are up for all peripherals, the central will establish isochronous channels to both peripherals. LED2 on all three boards should be glowing.

6. Press **BTN1** on either *peripheral* side to send data to the central.

   LED2 glowing indicates that ISOC is established; therefore, the data is sent via ISOC. When the central receives the data, it toggles the LED2 state.

7. Press **BTN1** at the *central* side to send data to both peripherals.

   LED2 glowing indicates that ISOC is established; therefore, the data is sent via ISOC. When the peripherals receives the data, it toggles the LED2 state.

   Isochronous channel statistics are printed every 5s. Central side has seperate isoc_tx_count and isoc_rx_count for each peripheral. In an ideal environment, the isoc_tx_count[i] on the central side should equal to the isoc_rx_count of the peripheral[i] device. The isoc_rx_count[i] on the central side should equal to the isoc_tx_count of the peripheral[i] device.

**Be Noticed** - to be simple, when a CIS connection is lost, you need to reset all the boards to start over. CIS connection recovery is not implemented yet.

## Steps to enable BTSpy logs

**Note:** This feature is available only for CYW920829M2EVK-02.

1. Add airoc-hci-transport from library manager before enabling spy traces, check airoc-hci-transport [README.md](https://github.com/Infineon/airoc-hci-transport/blob/master/README.md) for more details. If airoc-hci-transport library is included in the application, it is recommended to initialize it (Call cybt_debug_uart_init()). If airoc-hci-transport library is present in the application, but you want to use retarget-io library to get application traces in Teraterm/putty, you need to set the ENABLE_AIROC_HCI_TRANSPORT_PRINTF MACRO value to 0 in the application. Otherwise printf messages of the application will not be visible.

#define ENABLE_AIROC_HCI_TRANSPORT_PRINTF 1

2. Navigate to the application Makefile and open it. Find the Makefile variable `ENABLE_SPY_TRACES` and set it to the value *1* as shown:
    ```
    ENABLE_SPY_TRACES = 1
    ```
3. Save the Makefile, and then build and program the application to the board.

4. Open the [ClientControl](https://github.com/Infineon/btsdk-host-apps-bt-ble/tree/master/client_control) application and make the following settings:
   - Set the baud rate to 3000000.
   - Deselect the flow control checkbox.
   - Select the serial port and click on an open port.
5. Launch the [BTSpy](https://github.com/Infineon/btsdk-utils/tree/master/BTSpy) tool.

6. Press and release the reset button on the board to get the BTSpy logs on the BTSpy tool.

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).


## Design and implementation

This code example demonstrates the use of Bluetooth&reg; Low Energy isochronous channels with Infineon AIROC&trade; CYW20829 Bluetooth&reg; devices to send generic data.

Bluetooth&reg; Low Energy isochronous channels were introduced in the Bluetooth&reg; Core Specification 5.2. Though highly touted as the pillar for new audio applications, isochronous channels can be used to send generic data and provides some advantages over traditional LE ACL connections. Since LE ACL connections can only support a minimum of 7.5ms and guarantees data delivery, it puts constraints on the amount of latency that can be supported. Given that LE isochronous channels are targeted towards time critical data delivery, it provides latency advantages and hence can open up opportunities for a broader range of applications.

The application uses a UART resource from the Hardware Abstraction Layer (HAL) to print debug messages on a UART terminal emulator. The UART resource initialization and retargeting of standard I/O to the UART port are done using the retarget-io library.

Upon reset, if the application was built with the `AUTO_PAIRING` macro defined, both the *peripheral* and *central* will initiate an LE connection. Once the LE connection is established, LED1 glows, and an isochronous channel is then established.

Note that when the LE connection is first connected, a connection interval of 30 ms is used that is able to balance between collisions with the isochronous channel setup vs the time it takes to set up the isochronous connection. That is, if the LE connection interval is smaller (7.5 ms, for example), there's a higher chance of collisions to occur when setting up isochronous channels, which can result in failure in setting up the isochronous channel establishment. On the other hand, if the LE connection interval is larger (1 second for example), there's a much less chance of collisions but the isochronous channel set up will take significantly longer because the LE connection is used for the setup procedure.

If the application was not built with the `AUTO_PAIRING` macro defined, pressing and releasing BTN1 on the *central* board will start scanning for a peripheral. Similarly, pressing and releasing BTN1 on the *peripheral* board will start advertisements. Once the peripheral is found, the *central* will create a LE connection and LED1 on both boards will glow once the link is up. Afterwards the *central* will establish an isochronous channel and LED2 on both boards will glow once the ISOC channel is up.

Pressing BTN1 at the *peripheral* side will cause a burst of data to be sent to the *central*. Similarly, Pressing BTN1 at the *central* side will cause a burst of data to be sent to the *peripheral*. Once the data is received on either side, the respective LED2 will be toggled indicating reception of data.


## Resources and settings

This section explains the ModusToolbox&trade; software resources and their configuration as used in this code example. Note that all the configuration explained in this section has already been done in the code example.

- **Device Configurator:** ModusToolbox&trade; stores the configuration settings of the application in the *design.modus* file. This file is used by the Device Configurator, which generates the configuration firmware. This firmware is stored in the application's *GeneratedSource* folder.

   By default, all applications in a workspace share the same *design.modus* file - i.e., they share the same pin configuration. Each BSP has a default *design.modus* file in the *mtb_shared\TARGET_<bsp name>\<version>\COMPONENT_BSP_DESIGN_MODUS* directory. It is not recommended to modify the configuration of a standard BSP directly.

   To modify the configuration for a single application or to create a custom BSP, see the [ModusToolbox&trade; user guide](https://www.infineon.com/ModusToolboxUserGuide). This example uses the default configuration. See the [Device Configurator guide](https://www.infineon.com/ModusToolboxDeviceConfig).

- **Bluetooth&reg; Configurator:** The Bluetooth&reg; peripheral has an additional configurator called the "Bluetooth&reg; Configurator" that is used to generate the Bluetooth&reg; LE GATT database and various Bluetooth&reg; settings for the application. These settings are stored in the file named *design.cybt*.

   Note that unlike the Device Configurator, the Bluetooth&reg; Configurator settings and files are local to each respective application. The services and characteristics added are explained in the [Design and implementation](#design-and-implementation) section. See the [Bluetooth&reg; Configurator guide](https://www.infineon.com/ModusToolboxBLEConfig).

**Table 1. Application resources**

| Resource  |  Alias/object     |    Purpose     |
| :------- | :------------    | :------------ |
| UART (HAL)|cy\_retarget\_io\_uart\_obj| UART HAL object used by Retarget-IO for Debug UART port|

<br>

## Related resources

Resources  | Links
-----------|----------------------------------
Code examples  | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Development kits | Select your kits from the [Evaluation Board Finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board) page.
Libraries on GitHub | [abstraction-rtos](https://github.com/Infineon/abstraction-rtos) - RTOS Abstraction Library <br> [clib-support](https://github.com/Infineon/clib-support) - CLib Support Library <br>  [btstack](https://github.com/Infineon/btstack) - BTSTACK <br> [btstack-integration](https://github.com/Infineon/btstack-integration) - BTSTACK Porting Layer <br> [core-lib](https://github.com/Infineon/core-lib) - Core Library <br> [core-make](https://github.com/Infineon/core-make) - Core GNU make build system <br> [freertos](https://github.com/Infineon/freertos) - FreeRTOS for Infineon MCUs <br> [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1) - Hardware Abstraction Layer <br> [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) - ModusToolbox&trade; CAT1A Peripheral Driver Library <br> [recipe-make-cat1b](https://github.com/Infineon/recipe-make-cat1b) - PSoC&trade; 6 GNU make build system
Middleware on GitHub | [btsdk-ble](https://github.com/Infineon/btsdk-ble) - BTSDK Bluetooth&reg; LE library <br> [modustoolbox-software](https://github.com/Infineon/modustoolbox-software) - Links to all ModusToolbox&trade; middleware
Tools  | [Eclipse IDE for ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) - ModusToolbox&trade; software is a collection of easy-to-use software and tools enabling rapid development with Infineon MCUs, covering applications from embedded sense and control to wireless and cloud-connected systems using AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices.

<br>


## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.



## Document history

Document title: *CE238453* – *AIROC&trade; CYW20829: Bluetooth&reg; Low Energy ISOC Central*

| Version | Description of change |
| ------- | --------------------- |
| 1.0.0   | New code example      |
| 1.0.1   | Support 2 peripherals |
| 1.1.0   | Update Code Example with BTSTACK4.1|

-------------------------------------------------------------------------------

(c) Cypress Semiconductor Corporation, 2023. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolbox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
