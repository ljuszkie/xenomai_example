/*
 * cross-link.c
 *
 * Userspace test program (Xenomai alchemy skin) for RTDM-based UART drivers
 * Copyright 2005 by Joerg Langenberg <joergel75@gmx.net>
 *
 * Updates by Jan Kiszka <jan.kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/pipe.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <rtdm/ipc.h>

#define MAIN_PREFIX   "main : "
#define WTASK_PREFIX  "write_task: "
#define RTASK_PREFIX  "read_task: "
#define FILE    "/dev/ttyS1"
int fd = -1;
#define STATE_FILE_OPENED          1
#define STATE_RTASK_CREATED        2
#define STATE_WTASK_CREATED        4
#define STATE_RPTASK_CREATED       8
#define STATE_WPTASK_CREATED       16

#define QUEUE_SIZE 255
#define MAX_MESSAGE_LENGTH 255

#define XDDP_PORT_LABEL  "xddp-demo"

unsigned int state = 0;

/*                           --s-ms-us-ns */
RTIME write_task_period_ns =    100000000llu;
RT_TASK write_task;
RT_TASK read_task;
RT_TASK write_proxy_task;
RT_TASK read_proxy_task;

static void fail(const char *reason)
{
  perror(reason);
  exit(EXIT_FAILURE);
}

static int close_file( int fd, char *name)
{
  int err, i=0;
  do {
    i++;
    err = close(fd);
    switch (err) {
    case -EAGAIN:
      printf(MAIN_PREFIX "%s -> EAGAIN (%d times)\n",
	     name, i);
      rt_task_sleep(50000); /* wait 50us */
      break;
    case 0:
      printf(MAIN_PREFIX "%s -> closed\n", name);
      break;
    default:
      printf(MAIN_PREFIX "%s -> %s\n", name,
	     strerror(errno));
      break;
    }
  } while (err == -EAGAIN && i < 10);
  return err;
}
static void cleanup_all(void)
{
  if (state & STATE_FILE_OPENED) {
    close_file(fd, FILE);
    state &= ~STATE_FILE_OPENED;
  }

  if (state & STATE_WTASK_CREATED) {
    printf(MAIN_PREFIX "delete write_task\n");
    rt_task_delete(&write_task);
    state &= ~STATE_WTASK_CREATED;
  }

  if (state & STATE_WPTASK_CREATED) {
    printf(MAIN_PREFIX "delete write_proxy_task\n");
    rt_task_delete(&write_proxy_task);
    state &= ~STATE_WPTASK_CREATED;
  }

  if (state & STATE_RTASK_CREATED) {
    printf(MAIN_PREFIX "delete read_task\n");
    rt_task_delete(&read_task);
    state &= ~STATE_RTASK_CREATED;
  }
}

static void catch_signal(int sig)
{
  cleanup_all();
  printf(MAIN_PREFIX "exit\n");
  exit(1);
  return;
}

static void write_proxy_task_proc(void *arg) {
  RT_PIPE pipe;
  int retval;
  char msgBuf[MAX_MESSAGE_LENGTH];
  //task message blocks
  RT_TASK_MCB mymcb, talk_reply;

  // this is to debug which task started first
  printf("Entered write proxy\n");

  retval = rt_pipe_bind(&pipe, "demo", TM_NONBLOCK);
  if (retval)
    fail("pipe bind in write task");
  
  while (1) {

    retval = rt_pipe_read(&pipe, msgBuf, sizeof(msgBuf), TM_INFINITE);
    if (retval < 0) {
      printf("Error on pipe read");
      continue;
    }

    mymcb.data = msgBuf;
    mymcb.size = retval;
    
    talk_reply.size = 0;
    talk_reply.data = NULL;

    retval = rt_task_send(&write_task, &mymcb, &talk_reply, TM_NONBLOCK);

    if (retval < 0) {
      printf("Send error: %d\n", retval);
    } else {
      //printf("Send success!\n");
    }
  }
}


static void write_task_proc(void *arg)
{
  int err;
  int written = 0;

  int retval;
  char msgBuf[MAX_MESSAGE_LENGTH];
  RT_TASK_MCB mymcb, listen_reply;

  mymcb.data = (caddr_t)msgBuf;

  while (1) {
    /* receive message */
    mymcb.size = sizeof(msgBuf);
    retval = rt_task_receive(&mymcb, TM_INFINITE);

    if (retval < 0 ) {
      printf("Receiving error\n");
    }

    listen_reply.size = 0;
    listen_reply.data = NULL;
    rt_task_reply(retval, &listen_reply);

    written = write(fd, msgBuf, mymcb.size);
    if (written < 0 ) {
      printf(WTASK_PREFIX "error on write, %s\n",
	     strerror(errno));
      break;
    } else if (written != mymcb.size) {
      printf(WTASK_PREFIX "only %d / %zd byte transmitted\n",
	     written, mymcb.size);
      break;
    }
  }
 exit_write_task:
  if ((state & STATE_FILE_OPENED) &&
      close_file(fd, FILE " (write)") == 0)
    state &= ~STATE_FILE_OPENED;
  printf(WTASK_PREFIX "exit\n");
}

static void read_task_proc(void *arg)
{
  int err;
  int rd = 0;
  char read_buf [MAX_MESSAGE_LENGTH];
  ssize_t sz = sizeof(read_buf);
  memset(&read_buf, '\0', sz);

  int retval;
  //task message blocks
  RT_TASK_MCB mymcb, talk_reply;
  mymcb.data = (caddr_t) read_buf;

  while (1) {
    memset(&read_buf, '\0', sz);
    rd = read(fd, &read_buf, sz);
    if (rd > 0) {
      mymcb.size = rd;
      talk_reply.size = 0;
      talk_reply.data = NULL;
      retval = rt_task_send(&read_proxy_task, &mymcb, &talk_reply, TM_NONBLOCK);

      if (retval < 0) {
	printf("Send error: %d\n", retval);
      }

      // printf("%s", read_buf);
    } else if (rd < 0 ) {
      printf(RTASK_PREFIX "error on read, code %s\n", strerror(errno));
      break;
    }
  }
  if ((state & STATE_FILE_OPENED) &&
      close_file(fd, FILE " (read)") == 0)
    state &= ~STATE_FILE_OPENED;
  printf(RTASK_PREFIX "exit\n");
}

static void read_proxy_task_proc(void *arg) {
  int retval;
  char msgBuf[MAX_MESSAGE_LENGTH];
  RT_TASK_MCB mymcb, listen_reply;

  mymcb.data = (caddr_t)msgBuf;

  RT_PIPE pipe;
  int ret;

  ret = rt_pipe_bind(&pipe, "demo", TM_NONBLOCK);
  if (ret)
    fail("pipe bind in read task");
  
  while (1) {
    /* receive message */
    mymcb.size = sizeof(msgBuf);
    retval = rt_task_receive(&mymcb, TM_INFINITE);

    if (retval < 0 ) {
      printf("Receiving error\n");
    }

    listen_reply.size = 0;
    listen_reply.data = NULL;
    rt_task_reply(retval, &listen_reply);

    if (retval < 0 ) {
      printf("Receiving error\n");
    } else {
      msgBuf[mymcb.size] = '\0';
      printf("%s", mymcb.data);
      ret = rt_pipe_stream(&pipe, mymcb.data, mymcb.size);
      if (ret < 0)
	printf("Pipe send error");
    }
  }
  
}

void configure_serial(int *fd) {
  struct termios opt;

  /* Get port parameters */
  tcgetattr(*fd, &opt);
  /* Set port boudrate */
  cfsetispeed(&opt, B9600);
  cfsetospeed(&opt, B9600);

  /* 8N1 frame */
  opt.c_cflag &= ~PARENB;
  opt.c_cflag &= ~CSTOPB;
  opt.c_cflag &= ~CSIZE;
  opt.c_cflag |= CS8;
  /* Disable hardware flow control */
  opt.c_cflag &= ~CRTSCTS;
  /* Enable receiver and local mode */
  opt.c_cflag |= (CLOCAL | CREAD);
  /* Raw input */
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* Disable parity checking */
  opt.c_iflag &= ~(INPCK | ISTRIP);
  /* Don't map CR to NL or NL to CR */
  opt.c_iflag &= ~(ICRNL | INLCR);
  /* Don't map uppercase to lowercase */
  opt.c_iflag &= ~IUCLC;
  /* Don't ignore CR */
  opt.c_iflag &= ~IGNCR;
  /* Ignore BREAK condition */
  opt.c_iflag |= IGNBRK;
  /* Disable software flow control */
  opt.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* Raw output - other c_oflag bits ignored */
  opt.c_oflag &= ~OPOST;
  /* Set new settings */
  tcsetattr(*fd, TCSANOW, &opt);
}

int main(int argc, char* argv[])
{
  int err = 0;
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  /* open rtser0 */
  fd = open( FILE, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    printf(MAIN_PREFIX "can't open %s , %s\n", FILE,
	   strerror(errno));
    goto error;
  }
  state |= STATE_FILE_OPENED;
  printf(MAIN_PREFIX "file opened\n");

  configure_serial(&fd);

  mlockall(MCL_CURRENT|MCL_FUTURE);

  RT_PIPE pipe;
  int ret;

  ret = rt_pipe_create(&pipe, "demo", P_MINOR_AUTO, 0);
  if(ret)
    fail("pipe create");
  
  /* create write_task */
  err = rt_task_create(&write_task, "write_task", 0, 42, 0);
  if (err) {
    printf(MAIN_PREFIX "failed to create write_task, %s\n",
	   strerror(-err));
    goto error;
  }
  state |= STATE_WTASK_CREATED;
  printf(MAIN_PREFIX "write-task created\n");

  /* create write_proxy_task */
  err = rt_task_create(&write_proxy_task, "write_proxy_task", 0, 50, 0);
  if (err) {
    printf(MAIN_PREFIX "failed to create write_proxy_task, %s\n",
	   strerror(-err));
    goto error;
  }
  state |= STATE_WPTASK_CREATED;
  printf(MAIN_PREFIX "write-task created\n");

  /* create read_task */
  err = rt_task_create(&read_task, "read_task", 0, 51, 0);
  if (err) {
    printf(MAIN_PREFIX "failed to create read_task, %s\n",
	   strerror(-err));
    goto error;
  }
  state |= STATE_RTASK_CREATED;
  printf(MAIN_PREFIX "read-task created\n");

  /* create read_proxy_task */
  err = rt_task_create(&read_proxy_task, "read_proxy_task", 0, 51, 0);
  if (err) {
    printf(MAIN_PREFIX "failed to create read_proxy_task, %s\n",
	   strerror(-err));
    goto error;
  }
  state |= STATE_RPTASK_CREATED;
  printf(MAIN_PREFIX "read_proxy_task created\n");


  /* start write_task */
  printf(MAIN_PREFIX "starting write-task\n");
  err = rt_task_start(&write_task, &write_task_proc, NULL);
  if (err) {
    printf(MAIN_PREFIX "failed to start write_task, %s\n",
	   strerror(-err));
    goto error;
  }

  /* start write_proxy_task */
  printf(MAIN_PREFIX "starting write_proxy_task\n");
  err = rt_task_start(&write_proxy_task, &write_proxy_task_proc, NULL);
  if (err) {
    printf(MAIN_PREFIX "failed to start write_proxy_task, %s\n",
	   strerror(-err));
    goto error;
  }

  /* start read_proxy_task */
  printf(MAIN_PREFIX "starting read_proxy_task\n");
  err = rt_task_start(&read_proxy_task, &read_proxy_task_proc, NULL);
  if (err) {
    printf(MAIN_PREFIX "failed to start read_proxy_task, %s\n",
	   strerror(-err));
    goto error;
  }

  /* start read_task */
  printf(MAIN_PREFIX "starting read-task\n");
  err = rt_task_start(&read_task,&read_task_proc,NULL);
  if (err) {
    printf(MAIN_PREFIX "failed to start read_task, %s\n",
	   strerror(-err));
    goto error;
  }
  for (;;)
    pause();
  return 0;
 error:
  cleanup_all();
  return err;
}
