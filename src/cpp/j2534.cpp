#include <windows.h>
#include <cassert>
#include <cstdint>



struct PASSTHRU_MSG {
    uint32_t ProtocolID; /* vehicle network protocol */
    uint32_t RxStatus;   /* receive message status */
    uint32_t TxFlags;    /* transmit message flags */
    uint32_t Timestamp;  /* receive message timestamp(in microseconds) */
    uint32_t DataSize;   /* byte size of message payload in the Data array */
    uint32_t ExtraDataIndex;  /* start of extra data(i.e. CRC, checksum, etc) in
                                 Data array */
    unsigned char Data[4128]; /* message payload or data */
};

using PassThruOpen_t = int32_t (*)(void *, uint32_t *);
using PassThruClose_t = int32_t (*)(uint32_t);
using PassThruConnect_t = int32_t (*)(uint32_t, uint32_t, uint32_t, uint32_t,
                                      uint32_t *);
using PassThruDisconnect_t = int32_t (*)(uint32_t);
using PassThruReadMsgs_t = int32_t (*)(uint32_t, PASSTHRU_MSG *, uint32_t *,
                                       uint32_t);
using PassThruWriteMsgs_t = int32_t (*)(uint32_t, PASSTHRU_MSG *, uint32_t *,
                                        uint32_t);
using PassThruStartPeriodicMsg_t = int32_t (*)(uint32_t, const PASSTHRU_MSG *,
                                               uint32_t *, uint32_t);
using PassThruStopPeriodicMsg_t = int32_t (*)(uint32_t, uint32_t);
using PassThruStartMsgFilter_t = int32_t (*)(uint32_t, uint32_t,
                                             const PASSTHRU_MSG *,
                                             const PASSTHRU_MSG *,
                                             const PASSTHRU_MSG *, uint32_t *);
using PassThruStopMsgFilter_t = int32_t (*)(uint32_t, uint32_t);
using PassThruSetProgrammingVoltage_t = int32_t (*)(uint32_t, uint32_t, uint32_t);
using PassThruReadVersion_t = int32_t (*)(uint32_t, char *, char *, char *);
using PassThruGetLastError_t = int32_t (*)(char *);
using PassThruIoctl_t = int32_t (*)(uint32_t, uint32_t, void *, void *);



struct Library {
    void *handle;

    PassThruOpen_t PassThruOpen{};
    PassThruClose_t PassThruClose{};
    PassThruConnect_t PassThruConnect{};
    PassThruDisconnect_t PassThruDisconnect{};
    PassThruIoctl_t PassThruIoctl{};
    PassThruReadVersion_t PassThruReadVersion{};
    PassThruGetLastError_t PassThruGetLastError{};
    PassThruReadMsgs_t PassThruReadMsgs{};
    PassThruStartMsgFilter_t PassThruStartMsgFilter{};
    PassThruStopMsgFilter_t PassThruStopMsgFilter{};
    PassThruWriteMsgs_t PassThruWriteMsgs{};
    PassThruStartPeriodicMsg_t PassThruStartPeriodicMsg{};
    PassThruStopPeriodicMsg_t PassThruStopPeriodicMsg{};
    PassThruSetProgrammingVoltage_t PassThruSetProgrammingVoltage{};
};

extern "C" void *j2534_load(const char *path) {
    void *handle = LoadLibraryA(path);
    if (handle == nullptr) {
        return nullptr;
    }
    
    Library *lib = new Library;
    lib->handle = handle;
    if (!(lib->PassThruClose = reinterpret_cast<PassThruClose_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruClose"))))
        return nullptr;
    if (!(lib->PassThruOpen = reinterpret_cast<PassThruOpen_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruOpen"))))
        return nullptr;
    if (!(lib->PassThruConnect = reinterpret_cast<PassThruConnect_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruConnect"))))
        return nullptr;
    if (!(lib->PassThruDisconnect = reinterpret_cast<PassThruDisconnect_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruDisconnect"))))
        return nullptr;
    if (!(lib->PassThruIoctl = reinterpret_cast<PassThruIoctl_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruIoctl"))))
        return nullptr;
    if (!(lib->PassThruReadVersion = reinterpret_cast<PassThruReadVersion_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruReadVersion"))))
        return nullptr;
    if (!(lib->PassThruGetLastError = reinterpret_cast<PassThruGetLastError_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruGetLastError"))))
        return nullptr;
    if (!(lib->PassThruReadMsgs = reinterpret_cast<PassThruReadMsgs_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruReadMsgs"))))
        return nullptr;
    if (!(lib->PassThruStartMsgFilter = reinterpret_cast<PassThruStartMsgFilter_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStartmsgFilter"))))
        return nullptr;
    if (!(lib->PassThruStopMsgFilter = reinterpret_cast<PassThruStopMsgFilter_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStopMsgFilter"))))
        return nullptr;
    if (!(lib->PassThruWriteMsgs = reinterpret_cast<PassThruWriteMsgs_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruWriteMsgs"))))
        return nullptr;
    if (!(lib->PassThruStartPeriodicMsg = reinterpret_cast<PassThruStartPeriodicMsg_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStartPeriodicMsg"))))
        return nullptr;
    if (!(lib->PassThruStopPeriodicMsg = reinterpret_cast<PassThruStopPeriodicMsg_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStopPeriodicMsg"))))
        return nullptr;
    if (!(lib->PassThruSetProgrammingVoltage = reinterpret_cast<PassThruSetProgrammingVoltage_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruSetProgrammingVoltage"))))
        return nullptr;

    

    return reinterpret_cast<void*>(lib);
}

extern "C" void j2534_close(void *handle) {
    Library *lib = (Library*)handle;
    assert(lib);
    CloseHandle(lib->handle);
    delete lib;
}

extern "C" int32_t j2534_PassThruOpen(void *handle, const char *port, uint32_t *id) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruOpen((void*)port, id);
}

extern "C" int32_t j2534_PassThruClose(void *handle, uint32_t id) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruClose(id);
}

extern "C" int32_t j2534_PassThruConnect(void *handle, uint32_t DeviceID, uint32_t ProtocolID, uint32_t Flags, uint32_t Baudrate, uint32_t *pChannelID) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruConnect(DeviceID, ProtocolID, Flags, Baudrate, pChannelID);
}

extern "C" int32_t j2534_PassThruDisconnect(void *handle, uint32_t ChannelID) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruDisconnect(ChannelID);
}

extern "C" int32_t j2534_PassThruIoctl(void *handle, uint32_t HandleID, uint32_t IoctlID, void *pInput, void *pOutput) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruIoctl(HandleID, IoctlID, pInput, pOutput);
}

extern "C" int32_t j2534_PassThruReadVersion(void *handle, uint32_t DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruReadVersion(DeviceID, pFirmwareVersion, pDllVersion, pApiVersion);
}

extern "C" int32_t j2534_PassThruGetLastError(void *handle, char *pErrorDescription) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruGetLastError(pErrorDescription);
}

extern "C" int32_t j2534_PassThruReadMsgs(void *handle, uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruReadMsgs(ChannelID, pMsg, pNumMsgs, Timeout);
}

extern "C" int32_t j2534_PassThruStartMsgFilter(void *handle, uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, uint32_t *pMsgID) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruStartMsgFilter(ChannelID, FilterType, pMaskMsg, pPatternMsg, pFlowControlMsg, pMsgID);
}

extern "C" int32_t j2534_PassThruStopMsgFilter(void *handle, uint32_t ChannelID, uint32_t MsgID) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruStopMsgFilter(ChannelID, MsgID);
}

extern "C" int32_t j2534_PassThruWriteMsgs(void *handle, uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruWriteMsgs(ChannelID, pMsg, pNumMsgs, Timeout);
}

extern "C" int32_t j2534_PassThruStartPeriodicMsg(void *handle, uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pMsgID, uint32_t TimeInterval) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruStartPeriodicMsg(ChannelID, pMsg, pMsgID, TimeInterval);
}

extern "C" int32_t j2534_PassThruStopPeriodicMsg(void *handle, uint32_t ChannelID, uint32_t MsgID) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruStopPeriodicMsg(ChannelID, MsgID);
}

extern "C" int32_t j2534_PassThruSetProgrammingVoltage(void *handle, uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage) {
    Library *lib = (Library*)handle;
    assert(lib);
    return lib->PassThruSetProgrammingVoltage(DeviceID, PinNumber, Voltage);
}

extern "C" void *j2534_get(void *handle, const char *proc) {
    assert(handle);
    void *func = reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(handle), proc));
    return func;
}