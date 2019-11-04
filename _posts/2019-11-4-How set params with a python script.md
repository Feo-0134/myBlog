---
title: How to set params with a python script
published: true
---

# How to set params with a python script

```python
import sys

# print the name of the python script
print sys.argv[0] 

# print the first param
print sys.argv[1]
```

sys.arg[n] ( n>= 1 ) stands the params for the python script

### related functions

len(sys.argv)

```python
''' 要求该脚本满足以下条件：

1.通过-i -o选项来区别参数是输入文件还是输出文件.
>>> python convert.py -i inputfile -o outputfile

2.当不知道convert.py需要哪些参数时，用-h打印出帮助信息
>>> python convert.py -h

getopt函数原形:
getopt.getopt(args, options[, long_options])
'''

import sys, getopt


opts, args = getopt.getopt(sys.argv[1:], "hi:o:")
input_file=""
output_file=""
for op, value in opts:
    if op == "-i":
        input_file = value
    elif op == "-o":
        output_file = value
    elif op == "-h":
        usage()
        sys.exit()

'''
代码解释：

a) sys.argv[1:]为要处理的参数列表，sys.argv[0]为脚本名，所以用sys.argv[1:]过滤掉脚本名。

b) "hi:o:"
表示"h"是一个开关选项；"i:"和"o:"则表示后面应该带一个参数。

c) 调用getopt函数。函数返回两个列表：opts和args。

opts为分析出的格式信息。args为不属于格式信息的剩余的命令行参数。

opts是一个两元组的列表。每个元素为：(选项串,附加参数)。如果没有附加参数则为空串''。

getopt函数的第三个参数[, long_options]为可选的长选项参数，上面例子中的都为短选项(如-i -o)

长选项格式举例:

--version

--file=error.txt

让一个脚本同时支持短选项和长选项

getopt.getopt(sys.argv[1:], "hi:o:", ["version", "file="])
'''
```

