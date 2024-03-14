# JointCmdArray.msg

string[] names  
JointCmd[] cmds

# The Joint Command Array must be organised in the following format for a valid TX:
# cmds[0] = /l/pel
# cmds[1] = /l/hip
# cmds[2] = /l/kne
# cmds[3] = /l/ank
# cmds[4] = /r/pel
# cmds[5] = /r/hip
# cmds[6] = /r/kne
# cmds[7] = /r/ank
# 
# names may be the topicNamePrefix (e.g. 'l/hip'), being discussed now.
# 