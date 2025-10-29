# RL2025_HW01

# 🤖 RL2025HW01: Bring Up Your Robot (Armando 4-DOF Manipulator)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Distro](https://img.shields.io/badge/ROS%202%20Distro-SpecificareQui-brightgreen)](https://docs.ros.org/en/latest/index.html)
*(Modificare 'SpecificareQui' con la versione ROS 2 utilizzata, es. Humble, Foxy.)*

Questo repository contiene l'implementazione per l'Homework 01, focalizzato sulla configurazione, simulazione (in RViz2 e Gazebo) e controllo del manipolatore a 4 gradi di libertà **Armando** utilizzando **ROS 2**.

## 💡 Obiettivi Principali

* **Simulazione Dettagliata**: Visualizzazione in RViz2 (con collisioni box primitive) e simulazione fisica in Gazebo.
* **Controllo Avanzato**: Integrazione di `ros2_control` (Position Joint Interface) e implementazione di nodi C++ per il feedback e il controllo sequenziale (Position vs Trajectory controller).
* **Percezione**: Aggiunta di un sensore telecamera virtuale e configurazione del topic immagine.

## 📦 Pacchetti Disponibili in questo Repository

| Package | Descrizione |
| :--- | :--- |
| **`armando_description`** | Contiene il modello robotico (XACRO), le mesh, le configurazioni RViz e i file YAML per i controller. |
| **`armando_gazebo`** | Contiene i launch file per avviare la simulazione Gazebo e la configurazione del sensore telecamera. |
| **`armando_controller`** | Contiene l'implementazione del nodo C++ per il feedback dello stato dei giunti e l'invio dei comandi di posizione/traiettoria. |

---

## 🛠️ Getting Started (Setup)

Per avviare e testare il progetto, segui questi passi all'interno del tuo workspace ROS 2 (`~/ros2_ws`).

1.  **Clona il Repository:**
    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/P0l1702/RL2025HW01.git](https://github.com/P0l1702/RL2025HW01.git)
    ```

2.  **Compila e Sorsa:**
    ```bash
    cd ~/ros2_ws
    colcon build 
    source install/setup.bash
    ```

---

## 💻 Istruzioni per l'Uso

### 1. Launch del Manipolatore in RViz

Avvia la visualizzazione del robot con la GUI per la manipolazione manuale dei giunti. Le mesh di collisione sono state sostituite con box primitivi.

```bash
ros2 launch armando_description armando_display.launch.py

### 2. Launch del Manipolatore in Gazebo 🌍

Avvia l'ambiente di simulazione fisica **Gazebo** per caricare il modello del robot, l'interfaccia hardware (tramite `ros2_control`) e il bridging dei topic.

```bash
ros2 launch armando_gazebo armando_world.launch.py

3. Visualizzazione Sensore Telecamera 📷

Mentre Gazebo è in esecuzione, apri un nuovo terminale per visualizzare il feed video pubblicato dal sensore simulato (il topic è stato bridgeato e rimappato a /videocamera):
Bash

ros2 run rqt_image_view rqt_image_view

    Azione: Seleziona il topic corretto (es. /videocamera) in rqt_image_view.

4. Avvio/Gestione dei Controller

Avvia il controller desiderato tramite il controller_manager.

Avvio del Controller (Selettivo):
Bash

ros2 launch armando_controller armando_control.launch.py controller_type:=<type>

Sostituisci <type> con position o trajectory per avviare il controller desiderato.

Gestione/Monitoraggio:

    Lista Controller Attivi: ros2 control list_controllers

    Disattivazione di un Controller: ros2 control switch_controllers --stop <controller_name>

5. Avvio del Nodo Publisher/Subscriber

Avvia il nodo C++ per l'invio della sequenza di posizioni e la lettura dello stato dei giunti. Il nodo permette di selezionare tra il Position Controller e il Trajectory Controller tramite un argomento ROS.
Bash

ros2 run armando_controller arm_controller_node --ros-args -p use_trajectory:=<value>
