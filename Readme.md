# Instructions about how to use the kernel module
Hier we are providing some quick statrts for the onboarding to the kernel module. <br>
## Project Structure
Kernel file is under the folder "kernel module"

```
kernel_module
├── SXPFv.c                          
└── SXPFv.h
└── Makefile                  
└── README.md
└── app_for_kernel_module_test
```
All the developemnt for the kernel module is using C programm.
## Short instruction for the kernel module
In unix hardware devices are accessed ny the user through the kernel module, it has anotehr name "driver", which provicde API to ensure that the user can operate on the devcie. Normally the devices 
are groupped under the dictionary "/dev/{dev_name}". Currently only the charracter devcie is used, for this device systemcalls are directly go into the device kernel module. Once the device is registered 
in the filesystem, it must has the unique fixed identify number--major number and minor number. people must notice that the kernel spae and use space are isolated, that menas they can't interactly use the function 
of each otehr, in order to forward request from use to kernel system call must be applied, for the data transmission in kernel space there are two functions copy_to_user(...) and copy_from_user(...) are provided. 

## Quick Getting Started
  From the security issues administrator permission is needed when running the follwing command.
- Compile the kernel module <br>
  in the folder kernel module enter to the "lkm" then the Makefile will be there, use the command "make" to conpile when everything goes well then one file [name of kernel module].ko will be generated.  
- Load the kernel module <br>
  `insmod [name of kernel module].ko [options ...]`
- Unload the kerbnel module <br>
  `rmmod [name of kernel module] [options ...]`
- Debug the kernel using kernel log information <br>
  `dmesg [options ...]`
- More deteials you can refference the example and explanation about the character kernel module[Simple Linux character device driver](https://olegkutkov.me/2018/03/14/simple-linux-character-device-driver/) and [The Linux Kernel Module Programming Guide](https://sysprog21.github.io/lkmpg/#the-fileoperations-structure)

## Setting up the including Path on vscode
First ensure in vs code the relative extensions like "C/C++ extension for VS Code", "C/C++" are installed, and then use the hotkey "ctrl + shift + p", search "c/c++ edit configuations", open the file with extension .json(normally this filename is c_cpp_properties.json), and find the argument "includePath", add all the include path with komma isolation.
 - How to find the haed file location? <br>
 there are several ways to find the include path, but here only one method will be introduced. open your soruce code file and copy the name of headfile, in the command line type "locate -filename.h-", then all the corresponding file path which related to this file will be listed, choose the one which you think is most correct.
 - If you have one Makefile to compile the kernel module, you can also use the command `make V=1>[custome filename].txt` to get all to headfile dependencied, by which in this file try to find all the path that is marked down with "-I path/to/your/headfile", copy all these path to the "c/c++ edit configuations".
