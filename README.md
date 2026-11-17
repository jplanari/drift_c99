# drift_c99

Detecció de hotspots i tracking multi-objectiu per al projecte DRIFT (Hera, Core 1). Port en C99 estàtic del pipeline `postprocir_cpp`, que usava OpenCV.

## Requisits

- **Llenguatge:** ISO C99, sense `malloc`/`free` ni `string.h`.
- **Dependències:** només **libm de LibmCS** (ESA). No hi ha cap altra llibreria externa. `make check-deps` ho verifica.
- **Memòria:** totes les pools són estàtiques i de mida fixa, definides en temps de compilació.

## Estructura

```
include/
  drift_config.h   Dimensions, límits de pools i constants (sobreescrivibles amb -D)
  hs_detect.h      Detecció de hotspots (llindar, components, àrea de contorn, màxim de finestra)
  kf4.h            Filtre de Kalman de 4 estats (x, y, vx, vy)
  tracker.h        Tracker multi-objectiu
  hs_image.h       Normalització MINMAX a 8 bits
  drift_rng.h      Gaussià (Box-Muller amb libm)
  drift_util.h     zero/copy de bytes sense libc
src/               Implementacions corresponents
test/
  test_host.c      Proves unitàries de host
  golden_check.c   Comparació amb el golden master d'OpenCV
golden/
  golden_gen.cpp   Generador del golden master amb el codi original (OpenCV)
Makefile
```

## Pipeline per frame

1. `hs_detect()` llinda el frame (`> thr`), etiqueta components 8-connectats i retorna el centroide i l'àrea de contorn de cada component amb àrea ≥ `min_area`.
2. `tracker_update()` aplica el mateix ordre de passos que `KalmanTracker::update`:
   1. predicció de cada track;
   2. associació greedy per distància (amb coordenades truncades a enter, com l'original);
   3. correcció amb correcció per wrap-around, o coasting (sense mesura) i baixa després de `DRIFT_MAX_DISAPPEARED` frames;
   4. registre de les deteccions no associades;
   5. velocitat a partir de l'estat del filtre.

## Compilació i proves

Les proves de host enllacen amb una `libm.a` de LibmCS compilada per al host.

```
make LIBMCS_DIR=/ruta/a/libmcs test          # proves unitàries
make LIBMCS_DIR=/ruta/a/libmcs check-deps    # només dependències de libm
```

Per a una altra mida de frame (p. ex. TIRI 1024×768):

```
make LIBMCS_DIR=/ruta/a/libmcs EXTRA="-DDRIFT_IMG_W=1024 -DDRIFT_IMG_H=768" test
```

`LIBMCS_DIR` és el directori on s'ha compilat LibmCS (per defecte `/home/claude/libmcs_src/libmcs`). La `libm.a` del host és a `$(LIBMCS_DIR)/build-x86_64-linux-gnu/bin/libm.a`; per a SPARC cal indicar-la amb `LIBM=...`.

Requisits de compilació: `-std=c99 -Wall -Wextra -pedantic -Werror`, amb `-fno-tree-loop-distribute-patterns` i `-fno-stack-protector` (ja inclosos al `Makefile`). El primer evita que gcc converteixi els bucles de `drift_zero`/`drift_copy` en crides a `memset`/`memcpy`.

## Golden master contra OpenCV

Verifica que el port dona els mateixos resultats que el codi original. Cal OpenCV 4 i el repositori `postprocir_cpp` a la màquina.

```
export ORIG=$HOME/postprocir_cpp
g++ -std=c++11 -O2 -I$ORIG/src golden/golden_gen.cpp $ORIG/src/*.cpp \
    -o golden_gen $(pkg-config --cflags --libs opencv4)
mkdir -p golden_out && ./golden_gen golden_out

make LIBMCS_DIR=$HOME/libmcs golden-check GOLDEN_DIR=golden_out
```

`golden_gen` escriu els frames (`frame_NNN.bin`, float32) i les referències (`ref_det.csv`, `ref_trk.csv`). `golden_check` compara en dues etapes:

- **Detecció:** `hs_detect()` contra les deteccions de referència, per frame, ordenades per (y, x).
- **Tracker:** `tracker_update()` amb les deteccions de referència **en el seu ordre original** (els ids depenen de l'ordre), comparant id, posició i velocitat.

Toleràncies per defecte: 0,5 px per a deteccions i 1e-3 per al tracker. Es poden canviar com a arguments: `./golden_check golden_out <tol_det> <tol_trk>`.

## Semàntica respecte a OpenCV

Els punts següents són els que han de quadrar perquè el resultat sigui el mateix:

- **`KalmanFilter::predict`** calcula `statePre = F·statePost` i `errPre = F·errPost·Fᵀ + Q`, i després copia `statePost = statePre` i `errPost = errPre`. Així, un track sense mesura avança amb la seva velocitat.
- **`KalmanFilter::correct`** aplica `K = errPre·Hᵀ·(H·errPre·Hᵀ + R)⁻¹`, `statePost = statePre + K·(z − H·statePre)`, `errPost = (I − K·H)·errPre`.
- **`cv::contourArea`** del contorn extern s'aproxima amb el teorema de Pick: `A = N − B/2 − 1`, on `N` són els píxels del component i `B` els píxels de frontera (amb un veí 4-connectat fora del component o fora de camp).
- **`periodicDist`** rep `cv::Point` (enter), així que les coordenades es trunquen a enter abans de calcular la distància.
- **Llindar:** `THRESH_BINARY` és estricte (`> thr`).

## Limitacions conegudes

- **Forats:** els píxels d'un forat no s'afegeixen a l'àrea de contorn (pendent de confirmar contra `RETR_EXTERNAL`).
- **Centroide:** és el de píxels. Coincideix amb el dels moments poligonals per a formes simètriques, però pot diferir per a formes asimètriques.
- **Connectivitat:** 8.
- **Visualització:** `applyColorMap`, dibuix, GUI i vídeo no estan portats, perquè són eines del visor de host.
- **Format de frame:** TIRI (1024×768, 8–14 µm) està pendent de confirmar amb ESA. Les dimensions són configurables amb `DRIFT_IMG_W` i `DRIFT_IMG_H`.
- **Compilació per SPARC:** no verificada encara amb el toolchain creuat (BCC / `sparc-gaisler-elf-gcc`).

## Referències

- `claude/OpenCV_inventory_postProcIR.md`: inventari de funcions d'OpenCV i com s'han substituït.
- `claude/LibmCS_coverage.md`: què aporta LibmCS i què no.
- `claude/ESA_TIRI_clarification_questions.md`: preguntes obertes a ESA sobre TIRI.
