# Tesla-Model-3-Charger
Reverse Engineering of the Tesla Model 3 charger and development of an open source control board. PCB design files in DesignSpark 8.1 format.

25/06/19 : Thanks to a kind donation I will soon have a Model 3 battery charger for reverse engineering. Setting up a repo to contain the knowledge.

08/07/19 : Received the charger (corectly known as the Power Conversion System) as it also contains a DC DC converter. Also received a HV Controller and wiring looms from Model 3. Uploaded CAN logs from pcs and HV controller from the CP CAN bus. 500k.

To Power up the PCS : Connect 12V battery or power supply to the DC DC converter high power pins. Ground is pin nearest the edge. Then connect Wire W02 to the 12v battery ground. PCS will then wake up.


10/07/19 : Added more can logs

15/07/19 : so it seems I was wrong. The PCS does indeed transmit something by itself over IPC CAN Tx. Log uploaded using a Salea Logic analyser. Free software available here : https://www.saleae.com/downloads/

19/07/19 : Sucessfully interfaced with the IPC CAN using an SN65HVD255 CAN transciever. Uploaded new log files with IPC CAN captures. DCDC Converter section now waking up. Message id 0x22A is responsible for commanding the DCDC to startup. 

V1 design for a PCS controller done and released. Do not use this. It probably won't work....

20/07/19 : Excellent description of ipc can from a youtube comment :

Inter Process Communication (IPC) can use the CAN protocol over a half duplex single ended bus, without the usual Transceivers (that are used to convert to a differential bus topology for noise immunity and low EMI), by simply hardware OR'ing the TX lines onto a single data line. Used over short distances with system of a common ground it allows all the usual CAN goodness (built in checksums, automatic bus arbitration and multi-drop architecture) without the additional parts count for the full CAN physical layer. As Automotive spec micro-processors now pretty much all include multiple CAN nodes in hardware, it makes sense to use that rather than alternative IPC options (classic UART, SPI, I2C etc)  


See here: https://www.mikrocontroller.net/attachment/28831/siemens_AP2921.pdf

10/03/20 : Added some charging CAN logs from  a RHD 2019 M3.

08/03/21 : DCDC converter now up and running in the E46 touring project car. US version of the PCS. EU version also bench tested.

25/03/21 : Full PCS now up and running : https://vimeo.com/523413869 Full open source release of V2 STM32F103 based controller uploaded. WARNING : Not yet tested!

Uploaded a redacted version of my very hacky test code used on prototype Arduino/SAM3X8E based controller.

01/12/21 : Added fully functional V6 firmware for the Arduiono Due hardware. Tested and working in E46 touring with US PCS.

02/09/22 : Beta release for new type STM32 firmware V1.

22/09/22 : New firmware in action : https://vimeo.com/745900410

28/09/24 : Update on the PCS project : https://vimeo.com/1013243625
