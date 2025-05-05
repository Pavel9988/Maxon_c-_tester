

//aggiungere vettori booleani per controllo esecuzione funzioni
//divisione src e lib
//dividere controllo in pos assoluto e relativo 
// goto position  e velocity singolo
//sostituire cicli for con la mappa tra id e gli oggetti nodeObj
//per ora non mettere error handling
//fare versione senza controlli, movimentazione per un motore solo , diamo per scontato che stiamo accedendo ai nodi di cui disponiamo, anche senza messaggi di errore e ovviamente senza stampare feedback
#include "maxon.h"

using namespace std;
map <int, NodeObj> nodeMap;
vector <NodeObj> nodeList (1);
NodeObj node;
vector <int> scannedNodes = {0};
bool Appender;

Control::Control() {

}

void Control::SetParameters(string portName,string baudrate) {
    vector<char> vec(portName.begin(), portName.end());
    PortName=vec;
    vector<char> vec1(baudrate.begin(), baudrate.end());
    Baudrate=vec1;  
    vector<char> vec2(driver_type.begin(), driver_type.end());
    DeviceName=vec2;
    vector<char> vec3(Bus_type.begin(), Bus_type.end());
    ProtocolStackName=vec3;
    vector<char> vec4(a.begin(), a.end());
    connection_type_gateway=vec4;
    vector<char> vec5(b.begin(), b.end());
    connection_type_slaves=vec5;
}

bool Control::InitNetwork(string USB_Port, string BaudRate){
    std::cout << "Inizializzazione rete..." << USB_Port << std::endl;
    Control::SetParameters(USB_Port,BaudRate);//non equivalente all'init della comunicazione ma equivalente alla memorizzazione dei paramentri che verranno passati al momento dell'init con il master gateway
    
    for (unsigned int i = 0; i < 20; i++)
    {
        
        nodeList[0].handle = VCS_OpenDevice(DeviceName.data(), ProtocolStackName.data(),connection_type_gateway.data(), PortName.data(), errorCode);
        if (nodeList[0].handle != 0){
            std::cout << "connesso" << USB_Port << std::endl;
            std::cerr << "Init Network riuscito (nodo 0 connesso)" << std::endl;
            break;
        }else{
            std::cerr << "Init Network fallito (nodo 0 non connesso)" << std::endl;
        }
    }    
    
    nodeList[0].id = 0; //di default il master ha sempre nodo 0
    nodeMap[0].handle = nodeList[0].handle;
    // unsigned int Baudrate_uint = std::stoul(BaudRate);
    // std::cout << "Baudrate: " << Baudrate_uint << std::endl;
    // VCS_SetProtocolStackSettings(nodeList[0].handle,Baudrate_uint,10,errorCode);
    // VCS_SetGatewaySettings(nodeList[0].handle, Baudrate_uint, errorCode);
    //aggiungere controllo
    /*if (nodeList[0].handle == nullptr) {
        std::cerr << "Init Network fallito (nodo 0 non connesso)" << std::endl;
     } 
    return 0;*/
    return true;
}

bool Control::AreEqual(vector<int> v1, vector<int> v2) {
    if (v1.size() != v2.size()){
        return false;
    }

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    return v1 == v2;
}

bool Control::AreInside(vector<int> smaller, vector<int> bigger){
    if (smaller.size() > bigger.size()) {
        return false;
    }

    unordered_map<int, int> countBigger;

    // Conta quante volte ogni elemento appare in 'bigger'
    for (int val : bigger) {
        countBigger[val]++;
    }

    // Controlla che ogni elemento di 'smaller' sia presente in 'bigger'
    for (int val : smaller) {
        if (countBigger[val] == 0) {
            return false;  // elemento mancante o esaurito
        }
        countBigger[val]--;
    }

    return true;
}

void Control::ScanNodes() {
    
    unsigned short nodeId;
    unsigned int objectIndex = 0x1018; // Indirizzo dell'Object Dictionary per identificare i nodi
    unsigned char subIndex = 1;
    unsigned int nodeCount = 0;
    unsigned int bytesRead = 0; // Numero di byte letti

    cout << "Scansione dei nodi CANopen..." << endl;
    
    for (nodeId = 1; nodeId <= 12; nodeId++) { // CANopen supporta 128 nodi, il nodo 0 per convenzione lo assegnamo sempre al master
        unsigned int data;
        if (VCS_GetObject(nodeList[0].handle, nodeId, objectIndex, subIndex, &data, sizeof(data), &bytesRead, errorCode)) {
            cout << "Trovato nodo: " << nodeId << " (Bytes letti: " << bytesRead << ")" << endl;
            nodeCount++;
            scannedNodes.push_back(nodeId);
        }
    }
    scannedForNodes = 1;
    cout << "Numero subdevices rilevati: " << nodeCount << endl;
}

void Control::Connect(vector<int>& nodeVec) {

        nodeList.resize(nodeVec.size());
        for (unsigned int i=1; i<nodeVec.size(); i++){
        node.id = nodeVec[i];
        node.handle=VCS_OpenSubDevice(nodeList[0].handle, DeviceName.data(), connection_type_slaves.data(), errorCode);
        nodeList[i] = node;
        nodeMap[node.id]=nodeList[i];
        //aggiungere controllo errori fatto bene 
            if (node.handle == nullptr){
            cerr << "Nodo " + to_string(nodeMap[node.id].id) + " non connesso" << endl;
            }
            else {
            cerr << "Nodo " + to_string(nodeMap[node.id].id) + "  connesso" << endl;
            }
      
        }    
}

//aggiungere funzioni/e  get_property_node_pose(int pose_node); come return abbiamo un oggetto nodesProperties
//aggiungere funzioni/e get_property_node(int node_id);
void Control::EnableMotors(){
    
    for (unsigned int i=0; i<nodeList.size(); i++){
        //se il nodo i è dentro nodeVec....allora fai, altrimenti continue //anche qui osservazione scrita giù
        if (VCS_SetEnableState(nodeList[i].handle, nodeList[i].id, errorCode) != 0) {
            nodeList[i].enabled=true;
            cerr << "Nodo " + to_string(nodeList[i].id) + "  attivato" << endl;
        } else {
            nodeList[i].enabled=false;
            cerr << "Nodo " + to_string(nodeList[i].id) + " non attivato" << endl;
        }
            }
}

void Control::EnableMotor(int nodeId){
    for(unsigned int i = 0; i<=nodeList.size(); i++){
        if(nodeList[i].id == nodeId and nodeList[i].enabled == 0){
            if(VCS_SetEnableState(nodeList[i].handle, nodeList[i].id, errorCode)!=0){
            cerr << "Nodo " + to_string(nodeList[i].id) + "  attivato" << endl;
            }
            else{
            cerr << "Nodo " + to_string(nodeList[i].id) + "  NON attivato" << endl;
            }
            break;
        }        
    }
}

void Control::DisableMotors(){
        
    for (int i = nodeList.size(); i-- > 0;){
        //se il nodo i è dentro nodeVec....allora fai, altrimenti continue //per come è fatto il codice non posso cercare di fare il disable su nodi "sbalgiati"
        if (VCS_SetDisableState(nodeList[i].handle, nodeList[i].id, errorCode) != 0) {
            nodeList[i].disabled=true;
            cerr << "Nodo " + to_string(nodeList[i].id) + "  disattivato" << endl;
        } else {
            nodeList[i].disabled=false;
            cerr << "Nodo " + to_string(nodeList[i].id) + " non disattivato" << endl;
        }
            }
}

vector<int> Control::getIdValues(vector<NodeObj> nodeList){
    vector<int> idValues;
    for (const NodeObj& node : nodeList) {
       idValues.push_back(node.id);
   }
   return idValues;
   }

void Control::SetRel(vector<int> relNodes){
    vector<int> posVec (0);
    for (unsigned int i = 0; i<=nodeList.size(); i++){
        for(unsigned int j = 0; j<= relNodes.size(); j++){
         if (nodeList[i].id == relNodes[j]){
            posVec.push_back(relNodes[j]);
        }
        

        }
    }
    if(AreInside(relNodes,posVec) != 0){

    }
}

void Control::ActivatePositionProfileMode(vector<int>& nodeVec) {    
    cerr << "Nodi da mandare in profilo di posizione disponibili" << endl;
        for(unsigned int i=0;i<nodeVec.size();i++){
        VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id,1,errorCode);
        nodeMap[nodeVec[i]].Mode_Of_Operation=1;
                VCS_SetPositionProfile(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, 5000, 5000, 5000,errorCode);//fare una funzione dedicata in un secondo momento
                if (VCS_ActivateProfilePositionMode(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, errorCode) != 0 ) {
                nodeMap[nodeVec[i]].movementType=1;
                    cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in posizione attivato" << endl;
                } 
                else {
                    nodeMap[nodeVec[i]].movementType=0;
                    cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in posizione non attivato" << endl;
                }
            }
}

void Control::ActivatePositionMustMode(vector<int>& nodeVec) 
{    

    cerr << "Nodi da mandare in posizione MUST disponibili" << endl;
           for(unsigned int i=0;i<nodeVec.size();i++){
            VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id,-1,errorCode);
            nodeMap[nodeVec[i]].Mode_Of_Operation=-1;

            if (VCS_ActivatePositionMode(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, errorCode) != 0 ) {
                    nodeMap[nodeVec[i]].movementType=-1;
                       cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in posizione attivato" << endl;
                   } 
                    else {
                        nodeMap[nodeVec[i]].movementType=0;
                       cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in posizione non attivato" << endl;
                   }
               }
   }

void Control::ActivateProfileVelocityMode(vector<int>& nodeVec) {    
       cerr << "Nodi in velocità disponibili" << endl;
           for(unsigned int i=0;i<nodeVec.size();i++){
                VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id,3,errorCode);
                VCS_SetVelocityProfile(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, 5000, 5000, errorCode);//fare una funzione dedicata in un secondo momento
                if (VCS_ActivateProfileVelocityMode(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, errorCode) != 0 ) {
                    nodeMap[nodeVec[i]].movementType=3;
                    cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in profilo di velocità attivato" << endl;
                } 
                else {
                    nodeMap[nodeVec[i]].movementType=0;
                    cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in profilo velocità non attivato" << endl;
                   }
               }
   }

void Control::ActivateVelocityMustMode(vector<int>& nodeVec) {    
    cerr << "Nodi in velocità disponibili" << endl;
        for(unsigned int i=0;i<nodeVec.size();i++){
            VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id,-2,errorCode);
            for(unsigned int j=0;j<nodeVec.size();j++){
                Appender=VCS_ActivateVelocityMode(nodeMap[nodeVec[i]].handle,nodeMap[nodeVec[i]].id, errorCode);
                if (Appender != 0){
                    nodeMap[nodeVec[i]].movementType=-2;
                    cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in profilo di velocità attivato" << endl;
                    return;
                } 
            }            
            nodeMap[nodeVec[i]].movementType=0;
            cerr << "Nodo " + to_string(nodeMap[nodeVec[i]].id) + " controllo in profilo velocità non attivato" << endl;                
            }
}

void Control::GoToPosition(vector<int>& ids, vector<int>& targetPos) {
//aggiungere controllo 
    for (unsigned int i = 0; i<ids.size(); i++){               
                if(VCS_MoveToPosition(nodeMap[ids[i]].handle, nodeMap[ids[i]].id, targetPos[i],true,true,errorCode) == 0){
                    cerr << "Nodo " + to_string(nodeMap[ids[i]].id) + " NON mosso in posizione" << endl;
                }
                else{
                    cerr << "Nodo " + to_string(nodeMap[ids[i]].id) + " mosso in posizione" << endl;
                }
                continue;
    }
}

void Control::GoToPositionSingle(unsigned int& node, int& target){
    VCS_MoveToPosition(nodeMap[node].handle, nodeMap[node].id, target,true,true,errorCode);

    // VCS_SetPositionMust(nodeMap[node].handle, nodeMap[node].id, target,errorCode);
}

void Control::GoToVelocity(vector<int>& ids, vector<int>& targetVel) {
    //aggiungere controllo 
        for (unsigned int i = 0; i<ids.size(); i++){                
                    if(VCS_MoveWithVelocity(nodeMap[ids[i]].handle, nodeMap[ids[i]].id, targetVel[i],errorCode) == 0){
                        cerr << "Nodo " + to_string(nodeMap[ids[i]].id) + " NON mosso in velocità" << endl;
                    }
                    else{
                        cerr << "Nodo " + to_string(nodeMap[ids[i]].id) + " mosso in velocità" << endl;
                    }
                    continue;
            
        }
            
    }

void Control::GoToVelocityMustSingle(unsigned int& node, int& target) {

    VCS_SetVelocityMust(nodeMap[node].handle, nodeMap[node].id, target, errorCode);
}

void Control::GoToVelocityProfileSingle(unsigned int& node, int& target) {
    VCS_MoveWithVelocity(nodeMap[node].handle, nodeMap[node].id, target,errorCode);

}

void Control::CloseCommunication(){

    VCS_CloseAllDevices(errorCode);
 }

int Control::GetPosition(unsigned int& nodeId){
    VCS_GetPositionIs(nodeMap[nodeId].handle, nodeId, &feedback, errorCode);
        return feedback;
}

int Control::GetVelocity(unsigned int& nodeId){
        VCS_GetVelocityIs(nodeMap[nodeId].handle, nodeId, &feedback, errorCode);
        return feedback;
}



// NEW VERSION

// #include "maxon.h"

// using namespace std;

// map<int, NodeObj> nodeMap;
// vector<NodeObj> nodeList(1);
// NodeObj node;
// vector<int> scannedNodes = {0};

// // Constructor
// Control::Control()
// {

// }

// void Control::SetParameters(const string& portNameInput, const string& baudRateInput)
// {
//     portName.assign(portNameInput.begin(), portNameInput.end());
//     baudRate.assign(baudRateInput.begin(), baudRateInput.end());
//     deviceName.assign(driverType.begin(), driverType.end());
//     protocolStackName.assign(busType.begin(), busType.end());
//     connection_type_gateway.assign(connectionTypeGateway.begin(), connectionTypeGateway.end());
//     connection_type_slaves.assign(protocolType.begin(), protocolType.end());
// }

// // Utilities
// bool Control::AreEqual(vector<int>& v1, vector<int>& v2)
// {
//     if (v1.size() != v2.size())
//     {
//         return false;
//     }

//     sort(v1.begin(), v1.end());
//     sort(v2.begin(), v2.end());

//     return v1 == v2;
// }

// bool Control::AreInside(const vector<int>& smaller, const vector<int>& bigger)
// {
//     if (smaller.size() > bigger.size())
//     {
//         return false;
//     }

//     unordered_map<int, int> countBigger;

//     for (int val : bigger)
//     {
//         countBigger[val]++;
//     }

//     for (int val : smaller)
//     {
//         if (countBigger[val] == 0)
//         {
//             return false;
//         }
//         countBigger[val]--;
//     }

//     return true;
// }

// vector<int> Control::GetIdValues(const vector<NodeObj>& nodeList)
// {
//     vector<int> idValues;
//     for (const NodeObj& node : nodeList)
//     {
//         idValues.push_back(node.id);
//     }
//     return idValues;
// }

// void Control::SetRelativeMode(vector<int>& relNodes)
// {
//     vector<int> posVec(0);

//     for (unsigned int i = 0; i < nodeList.size(); i++)
//     {
//         for (unsigned int j = 0; j < relNodes.size(); j++)
//         {
//             if (nodeList[i].id == relNodes[j])
//             {
//                 posVec.push_back(relNodes[j]);
//             }
//         }
//     }
// }

// // Network initialization
// bool Control::InitNetwork(const string& USB_Port, const string& BaudRate)
// {
//     cout << "Initializing network..." << USB_Port << endl;
//     Control::SetParameters(USB_Port, BaudRate);

//     for (unsigned int i = 0; i < 20; i++)
//     {
//         nodeList[0].handle = VCS_OpenDevice(deviceName.data(), protocolStackName.data(), connection_type_gateway.data(), portName.data(), errorCode);
//         if (nodeList[0].handle != nullptr)
//             break;
//     }

//     if (nodeList[0].handle == nullptr)
//     {
//         cerr << "Network initialization failed (node 0 not connected)" << endl;
//         return 0;
//     } 
//     else
//     {
//         nodeList[0].id = 0; // The master always has node ID 0
//         nodeMap[0].handle = nodeList[0].handle;
//         cerr << "Network initialization successful (node 0 connected)" << endl;
//         return 1;
//     }
// }

// void Control::ScanNodes()
// {
//     unsigned short nodeId;
//     unsigned int objectIndex = 0x1018; // Object Dictionary address to identify nodes
//     unsigned char subIndex = 1;
//     unsigned int nodeCount = 0;
//     unsigned int bytesRead = 0;

//     cout << "Scanning CANopen nodes..." << endl;

//     for (nodeId = 1; nodeId <= 12; nodeId++)
//     {
//         unsigned int data;
//         if (VCS_GetObject(nodeList[0].handle, nodeId, objectIndex, subIndex, &data, sizeof(data), &bytesRead, errorCode))
//         {
//             cout << "Found node: " << nodeId << " (Bytes read: " << bytesRead << ")" << endl;
//             nodeCount++;
//             scannedNodes.push_back(nodeId);
//         }
//     }
//     scannedForNodes = 1;
//     cout << "Number of subdevices detected: " << nodeCount << endl;
// }

// void Control::Connect(vector<int>& nodeVec)
// {
//     nodeList.resize(nodeVec.size());

//     for (unsigned int i = 1; i < nodeVec.size(); i++)
//     {
//         node.id = nodeVec[i];
//         node.handle = VCS_OpenSubDevice(nodeList[0].handle, deviceName.data(), connection_type_slaves.data(), errorCode);
//         nodeList[i] = node;
//         nodeMap[node.id] = nodeList[i];

//         if (node.handle == nullptr)
//         {
//             cerr << "Node " + to_string(nodeMap[node.id].id) + " not connected" << endl;
//         }
//         else
//         {
//             cerr << "Node " + to_string(nodeMap[node.id].id) + " connected" << endl;
//         }
//     }    
// }

// void Control::EnableMotors()
// {
//     for (unsigned int i = 0; i < nodeList.size(); i++)
//     {
//         if (VCS_SetEnableState(nodeList[i].handle, nodeList[i].id, errorCode) != 0)
//         {
//             nodeList[i].enabled = true;
//             cerr << "Node " + to_string(nodeList[i].id) + " enabled" << endl;
//         }
//         else
//         {
//             nodeList[i].enabled = false;
//             cerr << "Node " + to_string(nodeList[i].id) + " not enabled" << endl;
//         }
//     }
// }

// void Control::EnableMotor(int nodeId)
// {
//     for (unsigned int i = 0; i < nodeList.size(); i++)
//     {
//         if (nodeList[i].id == nodeId && nodeList[i].enabled == 0)
//         {
//             if (VCS_SetEnableState(nodeList[i].handle, nodeList[i].id, errorCode) != 0)
//             {
//                 cerr << "Node " + to_string(nodeList[i].id) + " enabled" << endl;
//             }
//             else
//             {
//                 cerr << "Node " + to_string(nodeList[i].id) + " NOT enabled" << endl;
//             }
//             break;
//         }        
//     }
// }

// void Control::DisableMotors()
// {
//     for (int i = nodeList.size(); i-- > 0;)
//     {
//         if (VCS_SetDisableState(nodeList[i].handle, nodeList[i].id, errorCode) != 0)
//         {
//             nodeList[i].disabled = true;
//             cerr << "Node " + to_string(nodeList[i].id) + " disabled" << endl;
//         }
//         else
//         {
//             nodeList[i].disabled = false;
//             cerr << "Node " + to_string(nodeList[i].id) + " not disabled" << endl;
//         }
//     }
// }

// void Control::CloseCommunication()
// {
//     VCS_CloseAllDevices(errorCode);
// }

// // Movement Modes
// void Control::ActivatePositionProfileMode(vector<int>& nodeVec)
// {
//     cerr << "Nodes available for profile position control" << endl;
//     for (unsigned int i = 0; i < nodeVec.size(); i++)
//     {
//         VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, 1, errorCode);
//         nodeMap[nodeVec[i]].modeOfOperation = 1;
//         VCS_SetPositionProfile(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, 50000, 50000, 50000, errorCode);

//         if (VCS_ActivateProfilePositionMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, errorCode) != 0)
//         {
//             nodeMap[nodeVec[i]].movementType = 1;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " position profile control activated" << endl;
//         }
//         else
//         {
//             nodeMap[nodeVec[i]].movementType = 0;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " position profile control NOT activated" << endl;
//         }
//     }
// }

// void Control::ActivatePositionMustMode(vector<int>& nodeVec)
// {
//     cerr << "Nodes available for position MUST mode" << endl;
//     for (unsigned int i = 0; i < nodeVec.size(); i++)
//     {
//         VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, -1, errorCode);
//         nodeMap[nodeVec[i]].modeOfOperation = -1;

//         if (VCS_ActivatePositionMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, errorCode) != 0)
//         {
//             nodeMap[nodeVec[i]].movementType = -1;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " position control activated" << endl;
//         }
//         else
//         {
//             nodeMap[nodeVec[i]].movementType = 0;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " position control NOT activated" << endl;
//         }
//     }
// }

// void Control::ActivateProfileVelocityMode(vector<int>& nodeVec)
// {
//     cerr << "Nodes available for velocity profile control" << endl;
//     for (unsigned int i = 0; i < nodeVec.size(); i++)
//     {
//         VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, 3, errorCode);
//         VCS_SetVelocityProfile(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, 50000, 50000, errorCode);

//         if (VCS_ActivateProfileVelocityMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, errorCode) != 0)
//         {
//             nodeMap[nodeVec[i]].movementType = 3;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " velocity profile control activated" << endl;
//         }
//         else
//         {
//             nodeMap[nodeVec[i]].movementType = 0;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " velocity profile control NOT activated" << endl;
//         }
//     }
// }

// void Control::ActivateVelocityMustMode(vector<int>& nodeVec)
// {
//     cerr << "Nodes available for velocity control" << endl;
//     for (unsigned int i = 0; i < nodeVec.size(); i++)
//     {
//         VCS_SetOperationMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, -2, errorCode);

//         if (VCS_ActivateVelocityMode(nodeMap[nodeVec[i]].handle, nodeMap[nodeVec[i]].id, errorCode) != 0)
//         {
//             nodeMap[nodeVec[i]].movementType = -2;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " velocity control activated" << endl;
//         }
//         else
//         {
//             nodeMap[nodeVec[i]].movementType = 0;
//             cerr << "Node " + to_string(nodeMap[nodeVec[i]].id) + " velocity control NOT activated" << endl;
//         }
//     }
// }

// // Commands
// void Control::GoToPosition(vector<int>& ids, vector<int>& targetPos)
// {
//     for (unsigned int i = 0; i < ids.size(); i++)
//     {
//         if (VCS_MoveToPosition(nodeMap[ids[i]].handle, nodeMap[ids[i]].id, targetPos[i], true, true, errorCode) == 0)
//         {
//             cerr << "Node " + to_string(nodeMap[ids[i]].id) + " NOT moved to position" << endl;
//         }
//         else
//         {
//             cerr << "Node " + to_string(nodeMap[ids[i]].id) + " moved to position" << endl;
//         }
//     }
// }

// void Control::GoToPositionMustSingle(unsigned int& node, int& target)
// {
//     VCS_SetPositionMust(nodeMap[node].handle, nodeMap[node].id, target, errorCode);
// }

// void Control::GoToPositionProfileSingle(unsigned int& node, int& target)
// {
//     VCS_MoveToPosition(nodeMap[node].handle, nodeMap[node].id, target, true, true, errorCode);
// }

// void Control::GoToVelocity(vector<int>& ids, vector<int>& targetVel)
// {
//     for (unsigned int i = 0; i < ids.size(); i++)
//     {
//         if (VCS_MoveWithVelocity(nodeMap[ids[i]].handle, nodeMap[ids[i]].id, targetVel[i], errorCode) == 0)
//         {
//             cerr << "Node " + to_string(nodeMap[ids[i]].id) + " NOT moved in velocity" << endl;
//         }
//         else
//         {
//             cerr << "Node " + to_string(nodeMap[ids[i]].id) + " moved in velocity" << endl;
//         }
//     }
// }

// void Control::GoToVelocityMustSingle(unsigned int& node, int& target)
// {
//     VCS_SetVelocityMust(nodeMap[node].handle, nodeMap[node].id, target, errorCode);
// }

// void Control::GoToVelocityProfileSingle(unsigned int& node, int& target)
// {
//     VCS_MoveWithVelocity(nodeMap[node].handle, nodeMap[node].id, target, errorCode);
// }

// // Sensors feedback
// int Control::GetPosition(unsigned int& nodeId)
// {
//     int feedback;
//     VCS_GetPositionIs(nodeMap[nodeId].handle, nodeId, &feedback, errorCode);
//     return feedback;
// }

// int Control::GetVelocity(unsigned int& nodeId)
// {
//     int feedback;
//     VCS_GetVelocityIs(nodeMap[nodeId].handle, nodeId, &feedback, errorCode);
//     return feedback;
// }
