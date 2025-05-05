
#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <array>
#include <vector>
#include <map>
#include <algorithm>
#include <unordered_map>
using namespace std;

class NodeObj {

    public:

    void* handle;
    int id;//no.....questo va fuori
    bool connected = false;
    bool enabled = false;
    bool disabled = false;
    //0:not declared, 1:position mode, 2:velocity mode, 3:homing mode
    int movementType = 0;
    int Mode_Of_Operation;
   
    private:
};

class Control{

    public:

    Control();

    //effettua il connect sui nodi selezionati solo se sono gli stessi disponibili nella rete
    void  Connect(vector<int>& nodeVec);//------------------------------------------------------------------------------------2
    //effettua l'enable su tutti i nodi connessi
    void EnableMotors();//------------------------------------------------------------------------------------3
    //effettua l'enable su un solo motore
    void EnableMotor(int nodeId);
    void DisableMotors();
    bool InitNetwork(string USB_Port, string BaudRate);//------------------------------------------------------------------------------------1
    //variabile che indica se lo scan è stato eseguito
    bool scannedForNodes = 0;
    //funzione che confronta due vettori e controlla se hanno dentro gli stessi elementi
    bool AreEqual(vector<int> v1, vector<int> v2); 
    //funzione che controlla se gli elementi di un vettore sono tutti presenti all'interno di un altro vettore più grande
    bool AreInside(vector<int> smaller, vector<int> bigger);
    //imposta i parametri della classe
    void SetParameters(string portName,string baudrate);
    //imposta controllo in posizione relativo (0) o assoluto (1)
    void SetRel(vector<int> relNodes);
    void ActivatePositionProfileMode(vector<int>& nodeVec) ;//------------------------------------------------------------------------------------sterzi
    void ActivatePositionMustMode(vector<int>& nodeVec) ;
    void ActivateProfileVelocityMode(vector<int>& nodeVec);
    void ActivateVelocityMustMode(vector<int>& nodeVec);//------------------------------------------------------------------------------------trazione
    void GoToPosition(vector<int>& posVec, vector<int>& targetPosVec);
    void GoToPositionSingle(unsigned int& node, int& target);
    void ScanNodes();
    void GoToVelocity(vector<int>& ids, vector<int>& targetVel);
    void GoToVelocityProfileSingle(unsigned int& node, int& target);
    void GoToVelocityMustSingle(unsigned int& node, int& target);
    void CloseCommunication();
    int GetPosition(unsigned int& nodeId);
    int GetVelocity(unsigned int& nodeId);
    vector<int> getIdValues(vector<NodeObj> nodeList);
    void* handle;
    //costanti
    string a="USB";
    string b="CANopen";
    vector <char>connection_type_gateway;
    vector <char> connection_type_slaves;
    string driver_type ="EPOS4";
    string Bus_type="MAXON SERIAL V2";
    string Protocol_sub_devices="CANopen";
    //valori di campo dei tipi di connessione
    vector <char> DeviceName; 
    vector <char> ProtocolStackName;
    vector <char> PortName; 
    unsigned int *errorCode;
    vector <char> Baudrate;
    int feedback=0;

    //proviene da ros, nodi da comandare in posizione
    vector <int> posVec;
    //proviene da ros, nodi da comandare in velocità
    vector <int> velVec;
    //dimensione di posVec
    vector <int> targetPosVec;
    //dimensione di velVec
    vector <int> targetVelVec;

private:

    #ifndef MMC_SUCCESS
        #define MMC_SUCCESS 0
    #endif

    #ifndef MMC_FAILED
        #define MMC_FAILED 1
    #endif

    #ifndef Incoerence_numbernodes_number_setpoints
        #define Incoerence_numbernodes_number_setpoints 2
    #endif

    #ifndef MMC_MAX_LOG_MSG_SIZE
        #define MMC_MAX_LOG_MSG_SIZE 512
    #endif
};



// NEW VERSION

// #include <iostream>
// #include <string>
// #include <sstream>
// #include <unistd.h>
// #include <getopt.h>
// #include <cstdlib>
// #include <cstdio>
// #include <list>
// #include <cmath>
// #include <sys/types.h>
// #include <sys/times.h>
// #include <sys/time.h>
// #include <array>
// #include <vector>
// #include <map>
// #include <algorithm>
// #include <unordered_map>

// #include "Definitions.h"

// using namespace std;

// //-----------------------------------------------
// // Class representing a single motor node
// //-----------------------------------------------
// class NodeObj {
// public:
//     void* handle = nullptr;      // Communication handle
//     int id = -1;                 // Node ID
//     bool connected = false;      // Connection status
//     bool enabled = false;        // Motor enabled status
//     bool disabled = false;       // Motor disabled status

//     // Movement type:
//     // 0 - not declared
//     // 1 - position mode
//     // 2 - velocity mode
//     // 3 - homing mode
//     int movementType = 0;         

//     int modeOfOperation = 0;     // Protocol-specific operation mode

// private:
//     // No private members
// };

// //-----------------------------------------------
// // Class for managing and controlling multiple nodes
// //-----------------------------------------------
// class Control {
// public:
//     Control();

//     //----------------------
//     // Network Communication
//     //----------------------
//     bool InitNetwork(const string& usbPort, const string& baudRate);
//     void CloseCommunication();
//     void ScanNodes();

//     //----------------------
//     // Motor Connection and Control
//     //----------------------
//     void Connect(vector<int>& nodeVec);
//     void EnableMotors();
//     void EnableMotor(int nodeId);
//     void DisableMotors();

//     //----------------------
//     // Movement Modes
//     //----------------------
//     void ActivatePositionProfileMode(vector<int>& nodeVec);
//     void ActivatePositionMustMode(vector<int>& nodeVec);
//     void ActivateProfileVelocityMode(vector<int>& nodeVec);
//     void ActivateVelocityMustMode(vector<int>& nodeVec);

//     //----------------------
//     // Commands
//     //----------------------
//     void GoToPosition(vector<int>& posNodes, vector<int>& targetPositions);
//     void GoToPositionMustSingle(unsigned int& nodeId, int& targetPosition);
//     void GoToPositionProfileSingle(unsigned int& nodeId, int& targetPosition);
//     void GoToVelocity(vector<int>& velNodes, vector<int>& targetVelocities);
//     void GoToVelocityMustSingle(unsigned int& nodeId, int& targetVelocity);
//     void GoToVelocityProfileSingle(unsigned int& nodeId, int& targetVelocity);

//     //----------------------
//     // Getters
//     //----------------------
//     int GetPosition(unsigned int& nodeId);
//     int GetVelocity(unsigned int& nodeId);
//     vector<int> GetIdValues(const vector<NodeObj>& nodeList);

// private:

//     //----------------------
//     // Private Utilities
//     //----------------------
//     bool AreEqual(vector<int>& v1, vector<int>& v2);
//     bool AreInside(const vector<int>& smaller, const vector<int>& bigger);
//     void SetParameters(const string& portName, const string& baudRate);
//     void SetRelativeMode(vector<int>& relNodes);

//     //----------------------
//     // Private Members
//     //----------------------
//     void* handle = nullptr;
//     bool scannedForNodes = false;

//     // Device configuration constants
//     vector <char>connection_type_gateway;
//     vector <char>connection_type_slaves;
//     const string connectionTypeGateway = "USB";
//     const string protocolType = "CANopen";
//     const string driverType = "EPOS4";
//     const string busType = "MAXON SERIAL V2";
//     const string protocolSubDevices = "CANopen";

//     // Connection type fields
//     vector<char> deviceName = {};
//     vector<char> protocolStackName = {};
//     vector<char> portName = {};
//     vector<char> baudRate = {};

//     unsigned int* errorCode = nullptr;

//     // ROS-related input vectors
//     vector<int> posNodes = {};          // Nodes commanded in position
//     vector<int> velNodes = {};          // Nodes commanded in velocity
//     vector<int> targetPositions = {};   // Target positions
//     vector<int> targetVelocities = {};  // Target velocities

//     //----------------------
//     // Constants
//     //----------------------
//     #ifndef MMC_SUCCESS
//         #define MMC_SUCCESS 0
//     #endif

//     #ifndef MMC_FAILED
//         #define MMC_FAILED 1
//     #endif

//     #ifndef INCOHERENCE_NODES_SETPOINTS
//         #define INCOHERENCE_NODES_SETPOINTS 2
//     #endif

//     #ifndef MMC_MAX_LOG_MSG_SIZE
//         #define MMC_MAX_LOG_MSG_SIZE 512
//     #endif
// };

// //-----------------------------------------------