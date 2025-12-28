#ifndef SMS_DATA_H
#define SMS_DATA_H

#include <stdio.h>
#include <stdlib.h>

// --- GENERATEUR DE SMS V2 (4 Champs = 20 736 variantes) ---

// 1. L'accroche
static const char *sms_part_1[] = {
    "Salut,", "Hello,", "Urgent :", "Punaise,", "Dis-moi,",
    "Juste pour dire, ", "Au fait,", "S'il te plait,", "Note :",
    "Pfff ...", "Pour info,", "Hey ! "
};

// 2. Le sujet / L'action
static const char *sms_part_2[] = {
    "je suis bloque", "le test est OK", "il y a un souci",
    "tout est calme", "j'ai verifie", "le niveau est bas",
    "j'ai perdu", "on a recu", "il manque",
    "le capteur voit", "je repare", "ca fonctionne"
};

// 3. Le contexte / Le lieu / L'objet
static const char *sms_part_3[] = {
    "dans le garage", "avec la batterie", "vers la porte",
    "sur le parking", "dans la cuisine", "le colis",
    "les cles", "le dossier", "l'entree",
    "la temperature", "le code wifi", "la voiture"
};

// 4. La conclusion / L'appel à l'action
static const char *sms_part_4[] = {
    "- rappelle-moi.", "- a ce soir.", "- merci.",
    "- bisous.", "- a demain.", "- urgent.",
    "- sans faute.", "- c'est note.", "- bonne nuit.",
    "- bon courage.", "- A+.", "- tkt."
};

// Fonction d'assemblage
// Le buffer doit faire au moins 100 octets pour être large
void Generate_Random_SMS(char* buffer, size_t size) {
    int r1 = rand() % 12;
    int r2 = rand() % 12;
    int r3 = rand() % 12;
    int r4 = rand() % 12;

    // Exemple généré : "Salut, je suis bloque dans le garage - a ce soir."
    snprintf(buffer, size, "%s %s %s %s",
             sms_part_1[r1],
             sms_part_2[r2],
             sms_part_3[r3],
             sms_part_4[r4]);
}

#endif // SMS_DATA_H
