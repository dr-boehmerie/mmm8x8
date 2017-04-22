#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#if LINUX
#  include <termios.h>
#  include <sys/select.h>
#  include <errno.h>
#  include <unistd.h>
#endif

#if WIN
#  include <windows.h>
#  include <string.h>
#  include <malloc.h>
#endif

#define SERIAL_SRC 1
#include <serial.h>
#undef SERIAL_SRC

#if LINUX

int open_serial(char *serialport, SERHDL *hdl)
{
  int rc;
  struct termios options;

  if ((*hdl = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
  {
    rc = RET_SERIAL_ERR_OPEN;
    goto EXIT;
  }

  /* now set 38400,8,N,1 */
  /* get the current settings of the serial port */
  tcgetattr(*hdl, &options);

  /* set the read and write speed to 38400 BAUD */
  cfsetispeed(&options, B38400);
  cfsetospeed(&options, B38400);

  /* disable the parity bit */
  options.c_cflag &= ~PARENB;

  /* only one stop bit */
  options.c_cflag &= ~CSTOPB;

  /* 8-bits */
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  /* enable the receiver and set local mode */
  options.c_cflag |= (CLOCAL | CREAD);

  /* choosing raw input */
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  /* choosing raw output */
  options.c_oflag &= ~OPOST;
 
  /* now set the settings */
  if(tcsetattr(*hdl, TCSANOW, &options) == -1)
  {
    close(*hdl);
    rc = RET_SERIAL_ERR_SETATTR;
    goto EXIT;
  }

  rc = RET_SERIAL_OK;

EXIT:
  return rc;
}


int close_serial(SERHDL hdl)
{
  close(hdl);

  return RET_SERIAL_OK;
}



int read_serial(SERHDL hdl, unsigned char *buf, int count)
{
  int rc;
  unsigned char *pos;
  int nread;
  fd_set readfds;
  struct timeval timeout;

  /* check whether hdl is ready to read */
  FD_ZERO(&readfds);
  FD_SET(hdl, &readfds);
  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;
  rc = select(hdl + 1, &readfds, NULL, NULL, &timeout);
  if (rc == 0)
  {
    rc = -1;
    goto EXIT;
  }
  if (rc == -1)
  {
    goto EXIT;
  }
   
  /* now actually read */
  pos = buf;
  nread = count;
  do 
  {
    do
    {
      rc = read(hdl, pos, nread);
      if (rc != -1) 
      {
        pos = pos + rc;
        nread -= rc;
      }
    }
    while ((rc != -1) && (nread > 0));
  }
  while ((rc == -1) && (errno == EAGAIN));

EXIT:
  return rc;
}


int write_serial(SERHDL hdl, unsigned char *buf, int count)
{
  int rc;

  rc = write(hdl, buf, count);
  return rc;
}

#endif /* LINUX */

#if WIN

int open_serial(char *serialport, SERHDL *hdl)
{
  int rc;
  DCB dcb;
  COMMTIMEOUTS timeouts;
  char *windows_serialport;
#define DEVICE_PREFIX "\\\\.\\" 

  windows_serialport = calloc(strlen(serialport) + 
                              strlen(DEVICE_PREFIX) + 1, sizeof(unsigned char));
  if (windows_serialport == NULL)
  {
    rc = RET_SERIAL_ERR_OPEN;
    goto EXIT;
  }

  strcpy(windows_serialport, DEVICE_PREFIX);
  strncpy(windows_serialport + strlen(DEVICE_PREFIX), serialport,
          strlen("comxx"));

  if ((*hdl = CreateFile(windows_serialport, GENERIC_READ | GENERIC_WRITE, 0,
                         NULL, OPEN_EXISTING, 0, NULL)) == INVALID_HANDLE_VALUE)
  {
    rc = RET_SERIAL_ERR_OPEN;
    goto FREE_EXIT;
  }
 
  /* now set 38400,8,N,1 */
  /* get the current settings of the serial port */
  dcb.DCBlength = sizeof(dcb);
  if (GetCommState(*hdl, &dcb) == 0)
  {
    CloseHandle(*hdl);
    rc = RET_SERIAL_ERR_SETATTR;
    goto FREE_EXIT;
  }

  dcb.BaudRate = CBR_38400;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  if (SetCommState(*hdl, &dcb) == 0)
  {
    CloseHandle(*hdl);
    rc = RET_SERIAL_ERR_SETATTR;
    goto FREE_EXIT;
  }

  // Set COM port timeout settings
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  if(SetCommTimeouts(*hdl, &timeouts) == 0)
  {
    CloseHandle(*hdl);
    rc = RET_SERIAL_ERR_SETATTR;
    goto FREE_EXIT;
  }

  rc = RET_SERIAL_OK;

FREE_EXIT:
  free(windows_serialport); 

EXIT:
  return rc;
}


int close_serial(SERHDL hdl)
{
  int rc;

  CloseHandle(hdl);

  rc = RET_SERIAL_OK;
  return rc;
}


int read_serial(SERHDL hdl, unsigned char *buf, int count)
{
  int rc;
  DWORD ntoread;
  DWORD nread;

  ntoread = count;
  nread = 0;
  if (ReadFile(hdl, buf, ntoread, &nread, NULL) == FALSE)
  {
    rc = -1;
    goto EXIT;
  }
  
  rc = nread;

EXIT:
  return rc;
}


int write_serial(SERHDL hdl, unsigned char *buf, int count)
{
  int rc;
  DWORD ntowrite;
  DWORD nwritten;

  ntowrite = count;
  nwritten = 0;
  while (ntowrite - nwritten)
  {
    if (WriteFile(hdl, buf + nwritten, ntowrite, &nwritten, NULL) == FALSE)
    {
      rc = -1;
      goto EXIT;
    }
    ntowrite -= nwritten;
  }
  
  rc = count;

EXIT:
  return rc;
}

#endif /* WIN */
