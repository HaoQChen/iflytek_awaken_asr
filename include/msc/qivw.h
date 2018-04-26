#ifndef __QIVW_H__
#define __QIVW_H__

#ifdef __cplusplus
extern "C" {
#endif /* C++ */

#include "msp_types.h"

typedef int( *ivw_ntf_handler)( const char *sessionID, int msg, int param1, int param2, const void *info, void *userData );

const char* MSPAPI QIVWSessionBegin(const char *grammarList, const char *params, int *errorCode);
typedef const char* (MSPAPI *Proc_QIVWSessionBegin)(const char *grammarList, const char *params, int *errorCode);

int MSPAPI QIVWSessionEnd(const char *sessionID, const char *hints);
typedef int (MSPAPI *Proc_QIVWSessionEnd)(const char *sessionID, const char *hints);

int MSPAPI QIVWAudioWrite(const char *sessionID, const void *audioData, unsigned int audioLen, int audioStatus);
typedef int (MSPAPI *Proc_QIVWAudioWrite)(const char *sessionID, const void *audioData, unsigned int audioLen, int audioStatus);

int MSPAPI QIVWRegisterNotify(const char *sessionID, ivw_ntf_handler msgProcCb, void *userData);
typedef int (MSPAPI *Proc_QIVWRegisterNotify)(const char *sessionID, ivw_ntf_handler msgProcCb, void *userData);

int MSPAPI QIVWResMerge(const char *srcPath, const char *destPath, const char *params);
typedef int (MSPAPI *Proc_QIVWResMerge)(const char *srcPath, const char *destPath, const char *params);

int MSPAPI QIVWGetResInfo(const char *resPath, char *resInfo, unsigned int *infoLen, const char *params);
typedef int (MSPAPI *Proc_QIVWGetResInfo)(const char *resPath, char *resInfo, unsigned int *infoLen, const char *params);



#ifdef __cplusplus
} /* extern "C" */	
#endif /* C++ */

#endif /* __QIVW_H__ */