# moby
Dispositif électronique de détection de chute et localisation.
Ce code est pour un ESP32 (Lolin32 Lite) equipé d'un accéléromètre MPU6050.
La localisaiton se fait par WiFi Fingerprinting sur le serveur SySPAD.
La chute se faire par reconnaissance d'une signature d'accélération et une machine d'état.
Toutes les informations sont envoyés par WiFi.
Le dispositif permet aussi d'évaluer l'activité du résident (nombre de pas, temps d'activité, etc.).
Une batterie de 400mAh ainsi que une gestion optimale du deepSleep de l'ESP permet à la batterie de tenir 36h sans recharge
