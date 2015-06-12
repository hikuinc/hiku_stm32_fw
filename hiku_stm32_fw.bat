cd C:\Users\user\Documents\Hiku\hiku_stm32_fw\Objects
C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin hiku_stm32_fw.axf --output hiku_stm32_fw.bin
cscript /nologo ..\bin_to_string.vbs <hiku_stm32_fw.bin >hiku_stm32_fw.txt
curl https://agent.electricimp.com/Ew1EPudkAcf3/erase
curl --data-binary @hiku_stm32_fw.bin https://agent.electricimp.com/Ew1EPudkAcf3/push
curl https://agent.electricimp.com/M-yqdfdYxyCd/erase
curl --data-binary @hiku_stm32_fw.bin https://agent.electricimp.com/M-yqdfdYxyCd/push
curl https://agent.electricimp.com/1f0NdPTKcTN5/erase
curl --data-binary @hiku_stm32_fw.bin https://agent.electricimp.com/1f0NdPTKcTN5/push
curl https://agent.electricimp.com/9IBxSsVH2OIZ/erase
curl --data-binary @hiku_stm32_fw.bin https://agent.electricimp.com/9IBxSsVH2OIZ/push
curl https://agent.electricimp.com/gZlt1iwGP9Ql/erase
curl --data-binary @hiku_stm32_fw.bin https://agent.electricimp.com/gZlt1iwGP9Ql/push
