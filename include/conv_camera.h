
// ***************************************************************************************************************************************************************
// @ Autor: Moacir Wendhausen    
// @ Projeto: VORIS
// @ Data: 15/05/2026
//
// Função: 
//
// Função: Este arquivo contém a implementação da classe ConvCamera, que é responsável por controlar a câmera convencional usando o SDK Spinnaker da FLIR. 
// ***************************************************************************************************************************************************************


#pragma once


#include "Spinnaker.h"
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/opencv.hpp>

// Esta classe efetua o controle da camera convencional:
class ConvCamera {
private:
    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;
    
    // Contador de imagens salvas:
    int ct_imagens_salvas;

    // Dimensões do sensor da camera convencional:
    int64_t Width{0};
    int64_t Height{0};

    // Cria objeto system_ptr para o sistema e interfaces da camera convencinal:
    Spinnaker::SystemPtr system_ptr; 

    // Cria objeto cam_ptr para a camera convencional:
    Spinnaker::CameraPtr cam_ptr;

    // Variáveis da thread vinculada a aquisição:
    std::thread thread_captura;
    std::atomic<bool> hab_captura_stream{false};
    std::mutex mutex_frame;
    cv::Mat frame_buffer;

    // Controle de Salvamento que possibilta gerar trigger por HW
    std::atomic<bool> hab_salvar_imagem{false};
    std::string path_salvar_imagem;

    // Função membro private que captura imagem:
    Spinnaker::ImagePtr capturaImagem();

public:
    ConvCamera();
    virtual ~ConvCamera();

    // Função membro que faz a busca e conecta a camera convencional pelo nº de série:
    bool conecta(std::string serialNumber);

    cv::Mat getUltimoFrame();

    // Incilaiza a camera para acessa-la:
    bool inicializa();

    // FInaliza a camera de forma segura:
    void finaliza();

    void exibeModeloCamera();
    
    bool isOk();

    std::string getSerial();
    
    // Altera o modo entre Contínuo e Trigger HW:
    void setModoTrigger(bool habilitar);

    // Configura como trigger de etrada:
    void configuraInputTrigger();

    // Configura os principais paramentros da camera: shutter e gain:
    void configuraParametros(const float shutter, const float gain);

    // Controle da Thread e resgate do frame
    void iniciaThreadCaptura();

    void stopCapturaPorThread();

    void liberaSalvarImagem(std::string path_dir_to_save, bool teste=false); 

    void verParametros();

    // Metodos usados para cropar o sensor de imagem:
    void cropSensor();

    int64_t getWidth(){
        return Width;
    }; 
    
    int64_t getHeight(){
        return Height;
    }; 
 
};