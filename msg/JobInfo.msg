string job_id           # Unique ID of the Job.
string status            # Status of the Job.
string priority          # Priority of this job; 1=LOW, 2=MEDIUM, 3=HIGH, 4=CRITICAL.
string mex_id           # The Unique ID of the allocated MEx, if allocated.
uint8 task_count        # Number of tasks the Job has.
uint8 current_task      # The index of the current task that is active, if job is active.
TaskInfo[] task_list    #  The list of TaskInfo.msg with status and type of each task for this job.