# SDRAM Contoller
**Author: Hermann Zoha 2026**

## Module 

```{doxygenclass} m_sdram_controller
:project: sdram_controller
```

## State-Mashine

### 1. Overview
The SDRAM (Synchronous Dynamic Random-Access Memory) controller utilizes a synchronous finite state machine (FSM) to manage memory bank activation, data access, and overhead operations such as refreshing and precharging. 

Because SDRAM relies on internal analog operations (e.g., charging bitlines, sense amplifier settling), state transitions cannot happen instantaneously. The memory controller must insert **NOP (No Operation)** commands as wait states to satisfy the device's specific timing parameters.

---

### 2. State Diagram

The following diagram illustrates the primary operational states and transitions of the SDRAM FSM:

```{figure} ./figures/sdram_controller/sdram_controller_state_mashine.svg
:name: Simple state machine
:width: 75%
:align: center
Simpel state machine of this sdram controller.
```
### 3. State Functional Descriptions

#### 3.1 START (Initialization)
* **Description:** The initial state upon hardware reset or power-up. 

##### 3.2 IDLE
* **Description:** The memory is unactivated, stable, and waiting for a command. All banks are precharged.

#### 3.3 REFRESH
* **Description:** Triggers an `AUTO REFRESH` command to maintain data integrity within the dynamic memory cells.


#### 3.4 ACTIVE
* **Description:** An `ACTIVATE` command opens a specific row in a selected bank, moving the row data into the sense amplifiers.

### 3.5 READ / WRITE
* **Description:** Column addresses are supplied to read from or write to the currently active row. Data burst transfers occur in these states.

### 3.6 PRECHARGE
* **Description:** Deactivates the open row in the selected bank (or all banks) and restores the bitlines to the mid-voltage level.

---

### 4. The Role of NOP (No Operation) Commands

SDRAM commands represent instantaneous clock-edge triggers, whereas the physical memory operations require distinct time intervals to complete. Therefore, **NOP commands must be continuously issued between state transitions** to serve as deterministic wait states. 

The table below outlines the specific timing constraints where NOPs must be inserted:

## State Command
 Summary of the SDRAM Controller Command Table

This table defines the command truth table used to manage the SDRAM memory.
Key Technical Details:
 -  **Inverted Logic:** All control ports operate on inverted (active-low) logic. 
    A logical 'L' (Low) activates a signal, while 'H' (High) represents an inactive state.
 -  **Don't Care ('X'):** Signals marked with 'X' do not affect the command execution.

  | **Name**                 | **CS** | **RAW**   | **CAS**   | **WE** | **ADDR**  | **DQ** |
  |--------------------------|--------|-----------|-----------|--------|-----------|--------|
  | Command Inhibit (NOP)    | H      | X         | X         | X      | X         | X      |
  | No Operation (NOP)       | L      | H         | H         | H      | X         | X      |
  | Active                   | L      | L         | H         | H      | Ba/ROW    | X      |
  | Read                     | L      | H         | L         | H      | Ba/Col    | X      |
  | Write                    | L      | H         | L         | L      | Ba/Col    | X      |
  | Prechage                 | L      | L         | H         | L      | Code      | X      |
  | Auto refresh             | L      | L         | L         | H      | X         | X      |
  | Load Mode Register       | L      | L         | L         | L      | Op-code   | X      |

   ### Signal Glossary (Corrections applied):

   - CS    =  Chip Selekt
   - RAW   =  Raw Active Select
   - CAS   =  Colomne Active Select
   - WE    =  Write Enable
   - ADDR  =  Address
   - DQ    =  Data In/Out
   - Ba    =  Bank
   - Col   =  Colomne

```{doxygengroup} sdram_controller_defs
:project: sdram_controller
:content-only:
```