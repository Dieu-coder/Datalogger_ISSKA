#include <Wire.h>  // Inclure la bibliothèque Wire pour la communication I2C

#define EZO_PMP_I2C_ADDRESS 0x67 // L'adresse I2C de la pompe EZO-PMP (vérifiez cette adresse)

void setup() {
  Serial.begin(115200);  // Initialisation du moniteur série pour affichage des messages
  Wire.begin();          // Initialisation de la communication I2C
}

void loop() {
  sendI2CCommand("D,10\r");  // Envoyer la commande "d,10" à la pompe

  delay(30000);  // Attendre 10 secondes avant d'envoyer la commande suivante
}

void sendI2CCommand(String command) {
  Wire.beginTransmission(EZO_PMP_I2C_ADDRESS);  // Démarrer la communication I2C avec la pompe
  for (int i = 0; i < command.length(); i++) {
    Wire.write(command[i]);  // Envoyer chaque caractère de la commande
  }
  Wire.endTransmission();  // Terminer la transmission I2C
  Serial.println("Commande envoyée : " + command);  // Afficher la commande envoyée sur le moniteur série
}
