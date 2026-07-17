# legacy

`matlab_relay.py` — relay UDP (ROS2 -> socket 127.0.0.1:9999) usado quando a
comunicação DDS direta com o MATLAB ainda não funcionava. **Obsoleto**: a
arquitetura atual usa duas máquinas físicas na mesma rede e o MATLAB assina
/Gesture nativamente com ros2subscriber. Mantido apenas como referência
histórica — NÃO rodar junto com o sistema (risco de caminho duplicado).
