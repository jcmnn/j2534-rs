#include <windows.h>
#include <cassert>
#include <stdint.h>



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

typedef int32_t (WINAPI *PassThruOpen_t)(void *, uint32_t *);
typedef int32_t (WINAPI *PassThruClose_t)(uint32_t);
typedef int32_t (WINAPI *PassThruConnect_t)(uint32_t, uint32_t, uint32_t, uint32_t,
                                      uint32_t *);
typedef int32_t (WINAPI *PassThruDisconnect_t)(uint32_t);
typedef int32_t (WINAPI *PassThruReadMsgs_t)(uint32_t, PASSTHRU_MSG *, uint32_t *,
                                       uint32_t);
typedef int32_t (WINAPI *PassThruWriteMsgs_t)(uint32_t, PASSTHRU_MSG *, uint32_t *,
                                        uint32_t);
typedef int32_t (WINAPI *PassThruStartPeriodicMsg_t)(uint32_t, const PASSTHRU_MSG *,
                                               uint32_t *, uint32_t);
typedef int32_t (WINAPI *PassThruStopPeriodicMsg_t)(uint32_t, uint32_t);
typedef int32_t (WINAPI *PassThruStartMsgFilter_t)(uint32_t, uint32_t,
                                             const PASSTHRU_MSG *,
                                             const PASSTHRU_MSG *,
                                             const PASSTHRU_MSG *, uint32_t *);
typedef int32_t (WINAPI *PassThruStopMsgFilter_t)(uint32_t, uint32_t);
typedef int32_t (WINAPI *PassThruSetProgrammingVoltage_t)(uint32_t, uint32_t, uint32_t);
typedef int32_t (WINAPI *PassThruReadVersion_t)(uint32_t, char *, char *, char *);
typedef int32_t (WINAPI *PassThruGetLastError_t)(char *);
typedef int32_t (WINAPI *PassThruIoctl_t)(uint32_t, uint32_t, void *, void *);



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
    if (handle == NULL) {
        return NULL;
    }
    
    Library *lib = new Library;
    lib->handle = handle;
    if (!(lib->PassThruClose = reinterpret_cast<PassThruClose_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruClose"))))
        return NULL;
    if (!(lib->PassThruOpen = reinterpret_cast<PassThruOpen_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruOpen"))))
        return NULL;
    if (!(lib->PassThruConnect = reinterpret_cast<PassThruConnect_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruConnect"))))
        return NULL;
    if (!(lib->PassThruDisconnect = reinterpret_cast<PassThruDisconnect_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruDisconnect"))))
        return NULL;
    if (!(lib->PassThruIoctl = reinterpret_cast<PassThruIoctl_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruIoctl"))))
        return NULL;
    if (!(lib->PassThruReadVersion = reinterpret_cast<PassThruReadVersion_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruReadVersion"))))
        return NULL;
    if (!(lib->PassThruGetLastError = reinterpret_cast<PassThruGetLastError_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruGetLastError"))))
        return NULL;
    if (!(lib->PassThruReadMsgs = reinterpret_cast<PassThruReadMsgs_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruReadMsgs"))))
        return NULL;
    if (!(lib->PassThruStartMsgFilter = reinterpret_cast<PassThruStartMsgFilter_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStartMsgFilter"))))
        return NULL;
    if (!(lib->PassThruStopMsgFilter = reinterpret_cast<PassThruStopMsgFilter_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStopMsgFilter"))))
        return NULL;
    if (!(lib->PassThruWriteMsgs = reinterpret_cast<PassThruWriteMsgs_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruWriteMsgs"))))
        return NULL;
    if (!(lib->PassThruStartPeriodicMsg = reinterpret_cast<PassThruStartPeriodicMsg_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStartPeriodicMsg"))))
        return NULL;
    if (!(lib->PassThruStopPeriodicMsg = reinterpret_cast<PassThruStopPeriodicMsg_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruStopPeriodicMsg"))))
        return NULL;
    if (!(lib->PassThruSetProgrammingVoltage = reinterpret_cast<PassThruSetProgrammingVoltage_t>(GetProcAddress(reinterpret_cast<HMODULE>(handle), "PassThruSetProgrammingVoltage"))))
        return NULL;

    

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
