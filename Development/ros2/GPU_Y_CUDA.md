# GPU y CUDA para el contenedor (opcional)

El mensaje **"NVIDIA runtime no detectado"** significa que Docker no puede usar la GPU. El QCar virtual y Cartographer funcionan también sin GPU; la GPU sirve sobre todo para visión/ML si la usas más adelante.

## 1. En el host (tu PC con Linux)

### Drivers NVIDIA
- Si tienes tarjeta NVIDIA, instala los drivers oficiales:
  - Ubuntu: **Software & Updates → Additional Drivers** y elige el driver NVIDIA recomendado, o:
  - [NVIDIA Linux drivers](https://www.nvidia.com/Download/index.aspx)

### CUDA (solo si vas a compilar/usar cosas que lo piden)
- Para **solo ejecutar** el contenedor con GPU no hace falta instalar CUDA en el host.
- Si quieres CUDA en el host (desarrollo nativo):
  - [CUDA Toolkit Download](https://developer.nvidia.com/cuda-downloads) → elige Linux, tu distro y versión.

### NVIDIA Container Toolkit (necesario para que Docker use la GPU)
Así el contenedor puede usar la GPU sin instalar CUDA dentro del contenedor:

```bash
# Añadir el repo de NVIDIA
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Instalar
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configurar Docker para usar el runtime NVIDIA
sudo nvidia-ctk runtime configure --runtime=docker

# Reiniciar Docker
sudo systemctl restart docker
```

Para que el contenedor **use** la GPU al arrancar, define la variable antes de ejecutar:
```bash
export USE_NVIDIA_GPU=1
./run_t2.sh
```
(Si no defines `USE_NVIDIA_GPU`, el contenedor arranca sin GPU para evitar errores si los drivers no están.)

## 2. Resumen

| Objetivo | Qué hacer |
|----------|-----------|
| Que el contenedor use la GPU | Instalar **nvidia-container-toolkit** y configurar Docker (pasos de arriba). |
| Tener CUDA en el host | Instalar CUDA Toolkit desde la web de NVIDIA (opcional). |
| Solo usar QCar virtual + mapa | No es obligatorio tener GPU; todo corre en CPU. |
