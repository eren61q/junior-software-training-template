[controller_scheme.json](controller_scheme.json)

1. Gereksinimler

Aşağıdaki yazılımların yüklü olduğundan emin olun:
    WPILib VSCode (resmi FRC dağıtımı)
    JDK 11 veya WPILib ile gelen OpenJDK
    Git
    USB-C veya Ethernet ile RoboRIO bağlantısı
    REV Hardware Client

2. Projeyi Çalıştırma (Simulation / Desktop Mode)

Robotu simülasyonda denemek için:
    Projeyi VSCode ile açın.
    Command Palette’e girin (Ctrl+Shift+P / Cmd+Shift+P).
    “WPILib: Simulate Robot Code” seçeneğini çalıştırın.
    Shuffleboard üzerinden mekanizma, encoder ve PID değerlerini gözlemleyebilirsiniz.
    Simülasyon, robot kodunun gerçek fizik ortamı olmadan test edilmesini sağlar.

 3. RoboRIO’ya Deploy Etme

Gerçek robota kod göndermek için:

    Bilgisayarı robot ağına bağlayın (USB-B ya da takımın WiFi ağı).
    RoboRIO’nun imaged ve doğru team number ile yapılandırılmış olduğundan emin olun.
    VSCode’da:
    Sol taraftaki WPILib sekmesinden “Deploy Robot Code” seçin veya
    Command Palette → WPILib: Deploy Robot Code
    Deploy tamamlandığında robot otomatik olarak yeni kodla yeniden başlar.

 4. Driver Station Kurulumu

Robotu sürmek için:

    Driver Station uygulamasını açın.
    Team Number alanına takım numarasını yazın.
    RoboRIO "Communications" ve "Robot Code" ışıkları yeşil olmalı.
    Gerekli güvenlik kontrolleri yapıldıktan sonra robot Enabled edilebilir.

5. Kontroller ve Test Adımları

    Joysticklerin doğru tanındığını Driver Station’daki USB sekmesinden doğrulayın.
    SmartDashboard/Shuffleboard üzerinden sensör değerlerini ve mekanizma durumlarını izleyin.
    Her mekanizma için test adımları:
    Motor doğru yönde mi dönüyor?
    Encoder artış/azalış yönü doğru mu?
    Limit switch’ler doğru okunuyor mu?
    PID kontrolü stabil mi?
    Her bir mekanizma yük altında da ayrıca test edilmelidir.