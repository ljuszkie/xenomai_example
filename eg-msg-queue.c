/* Xenomai's message_queue example */

#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <native/task.h>
#include <native/queue.h>

#include <rtdk.h>

#define TASK_PRIO  99 /* Highest RT priority */
#define TASK_MODE  0  /* No flags */
#define TASK_STKSZ 0  /* Stack size (use default one) */


RT_TASK p_desc, c_desc;
RT_QUEUE queue;

void consumer (void *cookie)
{
    ssize_t len;
    void *msg;
    int err;
    RT_QUEUE q_desc;

    /* Bind to a queue which has been created elsewhere, either in
       kernel or user-space. The call will block us until such queue
       is created with the expected name. The queue should have been
       created with the Q_SHARED mode set, which is implicit when
       creation takes place in user-space. */


    err = rt_queue_bind(&q_desc,"Queue",TM_INFINITE);
    if (err) {rt_printf("Consumer: Unable to bind to queue"); exit(-1); }

    /* Collect each message sent to the queue by the queuer() routine,
       until the queue is eventually removed from the system by a call
       to rt_queue_delete(). */

    while ((len = rt_queue_receive(&q_desc,&msg,TM_INFINITE)) > 0)
    {
      rt_printf("Consumer: received message len=%d bytes, ptr=%p, s=%s\n",
             len,msg,(const char *)msg);
      rt_queue_free(&q_desc,msg);
    }

    /* We need to unbind explicitly from the queue in order to
       properly release the underlying memory mapping. Exiting the
       process unbinds all mappings automatically. */

    rt_queue_unbind(&q_desc);

    if (len != -EIDRM) exit (-1);
        /* We received some unexpected error notification. */

}

void producer(void *cookie)
{
    RT_QUEUE q_desc;
    static char *messages[] = { "hello", "world", NULL };
    int n, len;
    void *msg;
    int err;

    err = rt_queue_bind(&q_desc,"Queue", TM_INFINITE);
    if (err != 0)
    { rt_printf ("Producer: Unable to bind to queue%d\n",err); exit(-1); }
   
    for (n = 0; messages[n] != NULL; n++)
    {
      rt_printf ("Producer: Putting in queue %d %s\n",n, messages[n]);
      len = strlen(messages[n]) + 1;
        /* Get a message block of the right size. */
      msg = rt_queue_alloc(&q_desc,len);
      if (!msg) 
      { printf ("Producer: Unable to alloc queue %d\n", err); exit(-1); }
      
      strcpy(msg,messages[n]);
      rt_queue_send(&q_desc,msg,len,Q_NORMAL);
    }
}

int main (int argc, char *argv[])
{
    int err;

    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_print_auto_init(1);

    err = rt_queue_create(&queue, "Queue",80,10,Q_FIFO|Q_SHARED);
    if (err != 0) { printf ("main():Unable to create queue %d\n",err); exit(-1);}
    
    err = rt_task_create(&p_desc,"producer", TASK_STKSZ,TASK_PRIO,T_JOINABLE);
    err = rt_task_create(&c_desc,"consumer", TASK_STKSZ,TASK_PRIO,T_JOINABLE);
    rt_task_start(&p_desc,&producer,NULL);
    rt_task_start(&c_desc,&consumer,NULL);

    printf ("Joining\n");
    rt_task_join(&p_desc);
    //    rt_task_join(&c_desc);  
    printf ("Deleting\n");
    rt_task_delete(&p_desc);
    rt_task_delete(&c_desc);
}
 
