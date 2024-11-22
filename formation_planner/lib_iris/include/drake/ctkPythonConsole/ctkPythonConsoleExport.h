#ifndef __ctkPythonConsoleExport_h
#define __ctkPythonConsoleExport_h

#if defined(WIN32)

 #if defined(ctkPythonConsole_EXPORTS)
  #define CTK_PYTHON_CONSOLE_EXPORT __declspec( dllexport )
 #else
  #define CTK_PYTHON_CONSOLE_EXPORT __declspec( dllimport )
 #endif

#else
 #define CTK_PYTHON_CONSOLE_EXPORT
#endif

#endif // __ctkPythonConsoleExport_h
