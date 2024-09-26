/**
*****************************************************************************
**
**  File        : syscalls.c
**
**  Abstract    : System Workbench Minimal System calls file
**
** 		          For more information about which c-functions
**                need which of these lowlevel functions
**                please consult the Newlib libc-manual
**
**  Environment : System Workbench for MCU
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright System Workbench for MCU.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. System Workbench for MCU permit registered System Workbench(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the System Workbench for MCU toolchain.
**
*****************************************************************************
*/

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


/* Variables */
//#undef errno
extern int errno;
// #define MAX_STACK_SIZE 0x2000

#ifdef __GNUC__
extern int __io_putchar(int ch) __attribute__((weak));
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
extern int fputc(int ch, FILE *f) __attribute__((weak));
#endif
extern int __io_getchar(void) __attribute__((weak));

char *__env[1] = { 0 };
char **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	(void)(pid); (void)(sig);
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

int _read (int file, char *ptr, int len)
{
	(void)(file);
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = (char)__io_getchar();
	}

return len;
}

int _write(int file, char *ptr, int len)
{
	(void)(file);
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

caddr_t _sbrk(int incr)
{
	register char * stack_ptr asm("sp");
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _close(int file)
{
	(void)(file);
	return -1;
}


int _fstat(int file, struct stat *st)
{
	(void)(file);
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	(void)(file);
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	(void)(file);
	(void)(ptr);
	(void)(dir);
	return 0;
}

int _open(char *path, int flags, ...)
{
	(void)(path);
	(void)(flags);
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	(void)(status);
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	(void)(name);
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	(void)(buf);
	return -1;
}

int _stat(char *file, struct stat *st)
{
	(void)(file);
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	(void)(old); (void)(new);
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	(void)(name); (void)(argv); (void)(env);
	errno = ENOMEM;
	return -1;
}