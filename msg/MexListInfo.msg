#uint32 seq
time stamp                          # Timestamp of the message.

uint8 total_mex_number              # Total number of Mobile Executors
uint8 standby                       # Of which have the status STANDBY.
uint8 charging                      # Of which have the status CHARGING.
uint8 assigned                      # Of which have the status ASSIGNED.
uint8 executing_task                # Of which have the status EXECUTING_TASK.
uint8 error                         # Of which have the status ERROR.

MexInfo[] mex_list_info_array       # List of MexInfo.msg items containing MEx specific information.
#string[] mex_list_info_array