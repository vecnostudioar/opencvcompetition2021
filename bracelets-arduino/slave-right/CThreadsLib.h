// ------------------------------------------------------------------------------------------------------
// Libreria de Threads Cooperativos.
//
// Version: 1.1
// Ultima modificacion: 18-08-2018
// Marcos M. Correas - TRCOM - http://www.trcom.com.ar
// ------------------------------------------------------------------------------------------------------

#ifndef CThreads_h
#define CThreads_h

#define CThreadBegin()       static int __statusThread=0; static unsigned long __timerThread; switch(__statusThread) { case 0:

// Espera mientras la condicion se cumpla
#define CThreadWaitWhile(c)  __statusThread = __LINE__; case __LINE__: \
                            if((c)) return 2

// Duerme el thread por X milisegundos
#define CThreadSleep(c)      __timerThread = millis(); __statusThread = __LINE__; case __LINE__: \
                            if((millis() - __timerThread) < c) return 2

// Lanza otro thread y espera su finalizacion
#define CThreadWaitFor(thread)  __statusThread = __LINE__; case __LINE__: \
                            if((thread != 0)) return 2

// Libera el procesador, para ejecutar otro thread. 
#define CThreadYield()    __statusThread = __LINE__; return 2; case __LINE__:

// Termina el thread
#define CThreadFinish()     __statusThread = 0; return 0

#define CThreadEnd()        } __statusThread = 0; return 0

#endif
