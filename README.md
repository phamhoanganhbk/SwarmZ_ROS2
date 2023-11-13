$Install PX4
Installer la chaîne d'outils de développement PX4 afin d'utiliser le simulateur.
\begin{lstlisting}[language=bash]
$ cd
$ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
$ cd PX4-Autopilot/
$ make px4_sitl
\end{lstlisting}

\subsection{Intall ROS2 Humble}
\begin{lstlisting}[language=bash]
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update && sudo apt upgrade -y
$ sudo apt install ros-humble-desktop
$ sudo apt install ros-dev-tools
$ source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
\end{lstlisting}

\subsubsection{Télécharger et construire l'espace de travail ROS2}
Il s'agit d'un programme développé sur la plateforme ROS2 pour recevoir des signaux et envoyer des signaux de contrôle aux drones. 
Télécharger le fichier \textbf{SwarmZ\_ROS2}.
\begin{lstlisting}[language=bash]
$ cd ..
$ git clone https://github.com/phamhoanganhbk/SwarmZ_ROS2.git
\end{lstlisting}




Veuillez consulter le fichier SWARMz_Tutorial_v1_1_Simulation_ROS_2_Multi_Drones.pdf pour plus de détails
