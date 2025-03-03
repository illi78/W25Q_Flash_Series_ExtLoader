# W25Q Flash Series External Loader Configuration

## Steps to Set Up  

1. Copy the file **`W25Q_Flash_Series_ExtLoader.ioc`** to a folder named **`W25Q_Flash_Series_ExtLoader`**.  
2. Run the file **`W25Q_Flash_Series_ExtLoader.ioc`** using **STM32CubeMX**.  
3. Click the **Generate** button to create the project.  

---

## Copy the Required Files  

Copy the following files to the corresponding paths in your project:  

`/Core/Inc/Dev_Inf.h`

`/Core/Inc/W25Qxx.h`

`/Core/Src/Dev_Inf.c`

`/Core/Src/W25Qxx.c`

`/Core/Src/Loader_Src.c`

`/EL_linker.ld`




---

## Modify Project Settings  

### Linker Script  
Navigate to:  
**`<project_name> → Properties → C/C++ Build → Tool Settings → MCU/MPU GCC Linker → General → Linker Script (-T):`**  

Set the value to:  
${workspace_loc:/${ProjName}/EL_linker.ld}




### Post-Build Command  
Navigate to:  
**`<project_name> → Properties → C/C++ Build → Build Steps → Post-Build Steps → Command:`**  

Set the value to:  
cp "${BuildArtifactFileBaseName}.elf" "../W25Q64_STM32F103C8_SPI1x.stldr"




---

## Hardware Configuration  

This **External Loader** is configured for the **W25Q64** connected to a **Blue Pill** board via:  
- **PB3, PB4, PB5** for SPI communication  
- **PB14** for Chip Select (CS)  

---

## Final Steps  

1. Perform a **Build** with no errors.  
2. The generated file **`W25Q64_STM32F103C8_SPI1x.stldr`** will be created.  
3. Copy this file to the **CubeProgrammer External Loader** directory:  

\STM32CubeProgrammer\bin\ExternalLoader




4. It is recommended to move existing files in this folder to a subfolder, so only the new files are visible.  
5. Open **CubeProgrammer** and check the **External Loader (EL) screen** to see the imported file.  

> **Note:** The name displayed in CubeProgrammer is the one defined in **`Dev_Inf.c`**, not the filename in the file system.
