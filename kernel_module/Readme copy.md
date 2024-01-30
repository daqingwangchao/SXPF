# Instructions about how to use the kernel module
Hier we are providing some quick statrts for the onboarding to the kernel module. <br>
### Project Structure
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
### Short instruction for the kernel module
In unix hardware devices are accessed by user through the kernel module, it has anotehr name "driver", which provicde API to ensure that the user can operate on the devcie. Normally the devices. 
are groupped under the dictionary "/dev/{dev_name}". Currently only the character devcie is used, for this device systemcalls are directly go into the device kernel module. Once the device is registered. 
in the filesystem, it must has the unique fixed identify number--major number and minor number. people must notice that the kernel spae and use space are isolated, that menas they can't interactly use the function. 
of each otehr, in order to forward request from use to kernel system call must be applied, for the data transmission in kernel space there are two functions copy_to_user(...) and copy_from_user(...) are provided. 

### Quick Getting Started
  1. Compile and load costomized kernel module <br> 
  From the security issues administrator permission is needed when running the follwing command.
- Compile the kernel module <br>
  in the folder kernel module enter to the "lkm" then the Makefile will be there, use the command "make" to compile when everything goes well then one file [name of kernel module].ko will be generated.  
- Load the kernel module <br>
  `insmod [name of kernel module].ko [options ...]`
- Unload the kernel module <br>
  `rmmod [name of kernel module] [options ...]`
- Debug the kernel using kernel log information <br>
  `dmesg [options ...]`
- More deteials you can refference the example and explanation about the character kernel module[Simple Linux character device driver](https://olegkutkov.me/2018/03/14/simple-linux-character-device-driver/) and [The Linux Kernel Module Programming Guide](https://sysprog21.github.io/lkmpg/#the-fileoperations-structure).
2. Test kernel module by using application "sxpf_send.c" and "sxpf-ini-sequence.c" 
- Go to folder "app_for_kernel_module_test", you will see the structure under this folder: 
```
app_for_kernel_module_test
├──lib
├──src
├   └──├sxpf-init-sequence.c
├      ├sxpf_send.c
├      ├sxpf_receive.c
├──CMakeLists.txt
```
All source file for test is under folder "src", folder "lib" has library file which is used for link to the binary file after compling of source code. It can generated by runing cmake.
- how to use cmake to generate Makefile? <br>
In order to make the compile process clean, you should build a hiarachy, it is recommand that under folder "app_for_kernel_module_test" create a new folder to store the generated file by cmake, hier I create folder named "build", go into this file and run the command "cmake .."(because CMakeLists.txt is in last layer), then in this folder the correspnding files will be generated, and under these folder you can find one file named "Makefile.txt", that is what we need for the compile, just run the command "make" or "cmake --build .", then these "Makefile.txt" will be executed, after it finishes, one binary file to these source code will be generated under same folder. <br>
**Note**: any change for the compile request(for example: modify the source code path, the source file) please go to "CmakeLists.txt" to do the modification, please don't change anything directly in generated "Makefile.txt" 
- how to run application? <br>
Go to folder "build": 
1 ) sxpf_send.c: 
`./<your generated bilnary file> <card number> <channel number> <data source>`, for example: `./virtualization 1 0 ~/test_pircture.raw` <br>
For test we apply one picture with raw format, specify card number and channel number, in application system call "memory mapping" allocates one shared memory used by user space and kernel space, this memory buffer will then be filled with picture data and in this way hardware can directly process the data from user space. <br>
in this application the function **sxpf_release_frame(...)** is neccessary to understand, after frame is sending to the hardware, hardware will return back this received data to user space to confirm that data is actually receieved by hardware, to achieve this one system call **select()**(manage events R/W) is used.
2 ) sxpf-ini-sequence.c: 
`./<your generated bilnary file> <path to card> -port <port number> -init <path to init file> --execute <event number>`, for example: `./sxpf-init-sequence /dev/sxpf3 -port 0 -ini ../conf/replay.ini --execute 0,2` <br>
For more ussage of example sapplication please look at Datasheet "SX_proFRAME_QuickStartGuide" provided by company. <br>
3 ) Debug original kernel module provided by the company
Sometimes by development of your own kernel module you may need to see the output information of original kernel module by add some debug break points in it, afterwards you need to compile the original kernel again to make the new one working, to achieve this, you need to go to path ./src/libs/sxpf_ll/lin64/driver, under this folder you will find Makefile.txt which is used to compile the by company provided kernel module.   

### Setting up the including Path on vscode
First ensure in vs code the relative extensions like "C/C++ extension for VS Code", "C/C++" are installed, and then use the hotkey "ctrl + shift + p", search "c/c++ edit configurations", open the file with extension .json(normally this filename is c_cpp_properties.json), and find the argument "includePath", add all the include path with komma isolation.
 - How to find the head file location? <br>
There are several ways to find the include path, but here only one method will be introduced. open your source code file and copy the name of headfile, in the command line type "locate -filename.h-", then all the corresponding file path which related to this file will be listed, choose the one which you think is most correct.
 - If you have one Makefile to compile the kernel module, you can also use the command `make V=1>[custome filename].txt` to get all to headfile dependencied, by which in this file try to find all the path that is marked down with "-I path/to/your/headfile", copy all these path to the "c/c++ edit configuations".