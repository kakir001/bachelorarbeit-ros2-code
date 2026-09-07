# Transform doğrulama — robot-dokunma ölçümleri (2026-06-01 oturum 14)

Yöntem: detector BASE vs robot TCP temas (ground-truth). Δ = detector − tcp.

| Trial | Detector BASE (mm) | TCP temas (mm) | Δx | Δy | Δz | Not |
|-------|--------------------|----------------|----|----|----|-----|
| 1 | (150, -70, -5) | (125, -62, -14) | +25 | -8 | +9 | conf düşük (0.25-0.48); reverse yöntem (önce dokun, sonra ölç) |
| 2 | (176, -108, -9) | (135, -109, -9) | +41 | +1 | 0 | conf yüksek (0.79-0.85); forward yöntem; gripper temas oryantasyonu trial-1'den farklı |

## Analiz (2 trial)
- Δx: +25, +41 → her ikisinde de detector x'i GERÇEK temastan ileride (pozitif bias), ama sabit değil (büyüyor)
- Δy: -8, +1 → küçük, manuel dokunma gürültüsü içinde
- Δz: +9, 0 → küçük
- KONFOUND: iki dokunmada gripper oryantasyonu çok farklıydı (quaternion) + `tcp` frame'i tam parmak ucu değil → x farkının bir kısmı bu ofsetin dönmesinden, saf transform hatası değil.
- Sonraki: temiz sonuç için DİK (top-down) sabit oryantasyonla 1-2 dokunma daha; o zaman tcp x-y = parmak ucu x-y = vida x-y olur.

## KALİBRASYON iter-1 uygulandı (2026-06-01)
- Dosya: `src/mycobot_world/urdf/mycobot_world.urdf.xacro:152`
- Kamera origin x: **0.2575 → 0.2245** (-33mm = ortalama Δx)
- TF doğrulama: camera_color_optical_frame→robot_base x: 0.245 → **0.212** ✓ (-33mm)
- Build: mycobot_world --symlink-install OK
- y, z dokunulmadı (gürültü içinde)
- BEKLEYEN: temiz top-down dokunmayla artık (residual) hata ölçümü → fazla/eksik düzeltme ayarı
- UYARI: veri konfound'lu (gripper oryantasyonu + tcp-parmak ucu ofseti + robot servo ~30mm). iter-2 için kartezyen GUI + sabit vida ile DİK dokunma önerilir.
