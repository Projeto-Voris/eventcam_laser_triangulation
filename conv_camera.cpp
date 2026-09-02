// ***************************************************************************************************************************************************************
// @ Autor: Moacir Wendhausen    
// @ Projeto: VORIS
// @ Data: 15/05/2026
//
// Função: 
//
// Função: Este arquivo contém a implementação da classe ConvCamera, que é responsável por controlar a câmera convencional usando o SDK Spinnaker da FLIR. 
// A classe ConvCamera herda todos os métodos e atributos da classe Spinnaker::Camera, e sobrescreve os métodos de inicialização, captura de imagem, entre outros, 
// para adaptar às necessidades do projeto.
// ***************************************************************************************************************************************************************

#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>

#include "Spinnaker.h"

#include "conv_camera.h"


// Construtor: apenas inicializa membros que não existem na classe pai
ConvCamera::ConvCamera() {
    system_ptr = nullptr;
    cam_ptr = nullptr;
    inicializada = false;

    ct_imagens_salvas= 0;  
}


// Destruidor: a limpeza do objeto Camera é feita automaticamente 
// pelo gerenciamento de memória da SDK/SmartPointers.
ConvCamera::~ConvCamera() {
     finaliza();
}




// Esta função apenas exibe o modelo da camera na inicialização:
void ConvCamera::exibeModeloCamera() {
    // Verifica se o ponteiro interno existe e se a câmera está inicializada:
    if (!cam_ptr || !cam_ptr->IsInitialized()) {
        std::cout << "[Atenção] Câmera convencional ainda não inicializada ou desconectada. \n";
        return;
    }

    // GetNodeMap(): Acessa a estrutura de controle físico da câmera, como tempo de exposição, ganho, trigger, etc.
    // GenICam XML: A câmera possui um XML interno que atua como dicionário de endereços de hardware.
    // Node Map: É a árvore de parâmetros (nós) gerada na RAM do PC a partir desse XML.
    // Regra estrita: OBRIGATÓRIO chamar Init() antes. É o Init() quem baixa o XML via barramento 
    // e aloca o Node Map na memória. Chamar GetNodeMap() antes causa erro de ponteiro nulo.    
    auto& nodeMap= cam_ptr->GetNodeMap();
    
    Spinnaker::GenApi::CStringPtr ptrModel= nodeMap.GetNode("DeviceModelName");

    if (Spinnaker::GenApi::IsAvailable(ptrModel)) {
        std::cout << " - " <<  ptrModel->GetValue() << " ..... Nº Serial: " << cam_ptr->DeviceSerialNumber() << "\n";
    }
}



// Função membro que conecta a camera convencional: 
bool ConvCamera::conecta(std::string serialNumber) {
    // Captura a interface de HW da camera:
    system_ptr= Spinnaker::System::GetInstance();

    // Captura as cameras conectadas:
    Spinnaker::CameraList camList= system_ptr->GetCameras();

    // Atribui ao ponteiro cam_ptr a camera convencional pelo nº de série:
    cam_ptr= camList.GetBySerial(serialNumber);

    if (!cam_ptr.IsValid()) {
        camList.Clear();
        return false; 
    }

    // Obrigado a dar um clear na lista de cameras:
    camList.Clear();
    return true;
}


// Getters simplificados usando os métodos da classe pai
std::string ConvCamera::getSerial() {
    if (!cam_ptr || !cam_ptr.IsValid()) {
        return "Câmera não conectada.";
    }
    // Retorna o serial convertido para o padrão std::string do C++
    return std::string(cam_ptr->DeviceSerialNumber().c_str());
}



bool ConvCamera::isOk() {
    return cam_ptr->IsInitialized();
}



// Inica a conexão de controle entre o software e o hardware da câmera.
// A camera só pode ser manipulada, capturar imagens, etc, depois deste comando:
bool ConvCamera::inicializa() {
    try {
        // Chama a implementação original da SDK da FLIR:
        if (cam_ptr && cam_ptr.IsValid()) {
                cam_ptr->Init();

                // Garantir que a câmera inicie em modo Contínuo, free-run, ignorando estados anteriores:
                Spinnaker::GenApi::INodeMap& nodeMap = cam_ptr->GetNodeMap();
                Spinnaker::GenApi::CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
                if (Spinnaker::GenApi::IsWritable(ptrTriggerMode)) {
                    ptrTriggerMode->SetIntValue(ptrTriggerMode->GetEntryByName("Off")->GetValue());
                }            
                                
                // Chama função mebro para configurar o trigger de HW da camera:
                configuraInputTrigger();

                // cam_ptr->BeginAcquisition();

                inicializada= true;
                return true;
        }
        return false;
    }
    catch (const Spinnaker::Exception& e) {
        std::cerr << "[Erro] Falha ao inicializar câmera convencional: " << e.what() << "\n";
        return false;       
    }
}



// Encerra a conexão de controle exclusiva entre o software e o hardware da câmera:
// - Liberação de Acesso: libera a câmera no nível do sistema operacional e do barramento, 
//                        permitindo que ela fique imediatamente disponível para outros softwares, como o SpinView,
//                        ou para a próxima execução deste codigo.
// - Desalocação de Memória: Remove da memória RAM as estruturas de controle e o NodeMap, XML GenICam, 
//                           que haviam sido carregados durante o Init().
// - Encerramento de Comunicação: Fecha o canal bidirecional de comandos via USB3 ou GigE de forma segura.
void ConvCamera::finaliza() {
    // Derruba a thread antes de parar e fechar a camera 
    stopCapturaPorThread(); 

    // Sequência de fechamento totalmente encapsulada e segura
    if (inicializada && cam_ptr && cam_ptr->IsInitialized()) {
        // Finaliza o fluxo::
        cam_ptr->EndAcquisition();

        cam_ptr->DeInit();
        inicializada= false;
    }

    // Libera a memoria:
    cam_ptr= nullptr;

    // Desliga o barramento:
    if (system_ptr) {
        system_ptr->ReleaseInstance();
        system_ptr= nullptr;
    }
}



Spinnaker::ImagePtr ConvCamera::capturaImagem() {
    try {
        if (!cam_ptr || !cam_ptr->IsInitialized()){ 
            return nullptr;
        }   

        // Inicia o fluxo de dados do sensor para a memória:
        //cam_ptr->BeginAcquisition();

        // Captura o próximo frame disponível com timeout de 1000ms:
        Spinnaker::ImagePtr pImage= cam_ptr->GetNextImage(1000);

        if (pImage->IsIncomplete()) {
            std::cerr << "Imagem incompleta\n";
            // Importante!!!! Liberar antes de retornar:
            pImage->Release(); 
            return nullptr;
        }
        
        // Cria uma instância do processador de imagem
        Spinnaker::ImageProcessor processor;
               
        // Converter a imagem usando o processador, convert() retorna um novo ImagePtr:
        Spinnaker::ImagePtr convertedImage= processor.Convert(pImage, Spinnaker::PixelFormat_Mono8);             

        pImage->Release();

        return convertedImage;
    }
    catch (const Spinnaker::Exception& e) {
        // Ignora silenciosamente o Timeout, pois é o estado normal de espera do HW Trigger
        if (e.GetError() == Spinnaker::SPINNAKER_ERR_TIMEOUT || std::string(e.what()).find("Timeout") != std::string::npos) {
            return nullptr;
        }
        
        std::cerr << " [Erro Crítico] Camera convencional: " << e.what() << std::endl;
        return nullptr;
    }
}



// Tal como a camera de eventos, a convencional também deve efetuar a cpatura em background usando thread:
void ConvCamera::iniciaThreadCaptura() {
    if (hab_captura_stream) 
        return;
 
    // Inicializa a aquisição de frames pela camera convencional:
    try {
        if (cam_ptr && cam_ptr->IsValid()) {
            cam_ptr->BeginAcquisition();
        }
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "[Erro] Falha ao iniciar stream da câmera: " << e.what() << "\n";
        return; 
    }        
        
    hab_captura_stream= true;
    thread_captura= std::thread([this]() {
        while (hab_captura_stream) {
            Spinnaker::ImagePtr img_ptr= capturaImagem();
            
            // Ignora timeouts do Trigger HW ou frames incompletos:
            if (!img_ptr || img_ptr->IsIncomplete()) {
                if (img_ptr) 
                    img_ptr->Release();
                continue; 
            }

            // Converte para OpenCV:
            cv::Mat frame_tmp(img_ptr->GetHeight(), img_ptr->GetWidth(), CV_8UC1, img_ptr->GetData(), img_ptr->GetStride());
            cv::Mat frame_color;
            cv::cvtColor(frame_tmp, frame_color, cv::COLOR_GRAY2BGR);
            
            // Se colicitado salva a imagem em arquivo via Trigger de HW sincronizado com a camera de eventos:
            if (hab_salvar_imagem) {
                cv::imwrite(path_salvar_imagem, frame_color);
                std::cout << "[ConvCamera] Imagem salva em: " << path_salvar_imagem << std::endl;
                hab_salvar_imagem= false;
            }

            // Copia para o buffer interno protegido, este processo é protegido por Mutex:
            {
                std::unique_lock<std::mutex> lock(mutex_frame);
                frame_color.copyTo(frame_buffer);
            }
            
            img_ptr->Release(); 
        }
    });
}




// Retorna uma cópia do último frame disponível para a interface
cv::Mat ConvCamera::getUltimoFrame() {
    std::unique_lock<std::mutex> lock(mutex_frame);
    return frame_buffer.clone(); 
}


// Sinaliza para a thread salvar o próximo frame capturado.
// Etsa função menbro apenas monta o path e controla a habilitação de salvar ou não via thread:
// Se teste vier true seá salva uma imagem de teste, caso contrário será salva um imagem sincronizada
// com a camera de eventos com trigger por HW.
void ConvCamera::liberaSalvarImagem(std::string path_dir_to_save, bool teste) {
    std::string full_path= "";
    std::stringstream ss_pasta;

    // Obter data para o nome da pasta e do arquivo:
    auto agora= std::chrono::system_clock::now();
    auto tempo_t= std::chrono::system_clock::to_time_t(agora);
    struct tm *info= std::localtime(&tempo_t);

    if (teste){
        // gera o nome do diretório:   
        ss_pasta << path_dir_to_save.c_str() << std::put_time(info, "%d_%m_%Y") << "_teste";
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directories(nome_pasta);
        }

        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "image_" + time + ".png";
        full_path= nome_pasta + filename;
    }
    else{
        // gera o nome do diretório:
        ss_pasta << path_dir_to_save.c_str() <<  std::put_time(info, "%d_%m_%Y") << "/imagens";
        std::string nome_pasta= ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directories(nome_pasta);
        }

        std::string filename= "imagem_" + std::to_string(ct_imagens_salvas) + ".png";
        full_path= nome_pasta + '/' + filename;
        ++ct_imagens_salvas;
    }

    path_salvar_imagem= full_path;
    hab_salvar_imagem= true;
};



// Para a thread com segurança:
void ConvCamera::stopCapturaPorThread() {
    hab_captura_stream= false;
    if (thread_captura.joinable()) {
        thread_captura.join();
    }
}


// Esta função mnembro permite alternar entre o modo continuo, para exibição no Canvas, e o modo trigger de HW:
void ConvCamera::setModoTrigger(bool habilitar) {
    if (!cam_ptr || !cam_ptr->IsInitialized()) return;
    try {
        if (habilitar) {
            cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerModeEnums::TriggerMode_On);
            // std::cout << "[ConvCamera] Trigger HW ATIVADO (Aguardando pulso).\n";
        } else {
            cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerModeEnums::TriggerMode_Off);
            // std::cout << "[ConvCamera] Trigger HW DESATIVADO (Vídeo contínuo liberado).\n";
        }
    } catch (const Spinnaker::Exception& e) {
        std::cerr << "[Erro] Falha ao alternar modo de trigger: " << e.what() << "\n";
    }
}



// Função mebro que configurar p trigger de HW da camera convencional:
void ConvCamera::configuraInputTrigger(){
    try {
        // Desliga o trigger antes de configurar para evitar disparos acidentais ou travamentos:
        cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerModeEnums::TriggerMode_Off);

        // Define que o trigger será aplicado ao início de cada frame, FrameStart:
        cam_ptr->TriggerSelector.SetValue(Spinnaker::TriggerSelectorEnums::TriggerSelector_FrameStart);

        // Define a origem do trigger, Line0 é a entrada optoisolada padrão na Blackfly S:
        cam_ptr->TriggerSource.SetValue(Spinnaker::TriggerSourceEnums::TriggerSource_Line0);

        // Define a ativação:
        // Usar RisingEdge para capturar a imagem no instante exato da transição, borda de subida.
        cam_ptr->TriggerActivation.SetValue(Spinnaker::TriggerActivationEnums::TriggerActivation_RisingEdge); 

        // Usar LevelHigh se a exposição da câmera for controlada ativamente pela duração do pulso em nível alto.
        // Cam->TriggerActivation.SetValue(Spinnaker::TriggerActivationEnums::TriggerActivation_LevelHigh);

        // Ativar o trigger após todas as configurações estarem definidas:
        cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerModeEnums::TriggerMode_Off);

    } catch (const Spinnaker::Exception& e) {
        std::cerr << "\n[Erro] Não foi possível configurar Trigger de HW da Câmera Convencional: " << e.what() << "\n";
    }

}



// Efetua um getter na camera comvencional para ler os paramentros tempo de exposição e ganho:
void ConvCamera::verParametros(){
    float shutter= cam_ptr->ExposureTime.GetValue();
    float ganho= cam_ptr->Gain.GetValue();
    
    std::cout << " - Tempo de Exposição    = " << shutter << " us\n";
    std::cout << " - Ganho                 = " << ganho << " dB\n";    
}




// Configura os principais paramentros da camera: shutter e gain:
void ConvCamera::configuraParametros(const float shutter, const float ganho){

    // Antes de dar um set, efetua aleitura dos paramentros atuais e exibe no console:
    //std::cout << " \nParâmetros Atuais da Câmera Convencional:\n";
    // verParametros();

    try {
        // Primeiro ajusta o tempo de exposição:
        // Desabilita o ajuste automatico de tempo de exposição:
        cam_ptr->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);

        // Set o modo para Timed: 
        cam_ptr->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);

        // Testa os valroes de shutter max e shutter min:
        float shutter_aux= shutter;
        float shutter_max= cam_ptr->ExposureTime.GetMax();
        float shutter_min= cam_ptr->ExposureTime.GetMin();
        if (shutter_aux>shutter_max){
            shutter_aux= shutter_max;
        }
        if (shutter_aux<shutter_min){
            shutter_aux= shutter_min;
        }

        // Set o tempo de exposição com o valor definido no parametro shutter, o valor é em milisegundos: 
        cam_ptr->ExposureTime.SetValue(shutter_aux);

        // Segundo ajusta o ganhho 
        //  Desabilita o ajuste automatico de ganho:
        cam_ptr->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);
    
        float ganho_aux= ganho;

        // Testa valores max. e min. do ganho.
        float max_ganho= cam_ptr->Gain.GetMax();
        float min_ganho= cam_ptr->Gain.GetMin();
            
        if (ganho_aux > max_ganho) {
            ganho_aux= max_ganho;
        }
        
        if (ganho_aux < min_ganho) {
            ganho_aux= min_ganho;
        }    

        // Seta o valor do ganho definido em gain, valor em dB:
        cam_ptr->Gain.SetValue(ganho_aux);
    }
    catch (const Spinnaker::Exception& e) {
        std::cerr << "[Erro] Falha ao setar shutter e ganho na camera convencional: " << e.what() << "\n";
    }

    // Depois de dar um set, efetua aleitura dos paramentros gravados e exibe no console:
    verParametros();    
}




// Função mebro que permite cropar o sensor da camera convencional, reduzindo a área de captura e aumentando a taxa de quadros.
// Esta redução torna a camera convencional com as mesmas dimensões da câmera d eevenos SilkyEvCam.
void ConvCamera::cropSensor(){
    // Verficação de segurança antes de alterar as dimensões do sensor:
    if (!cam_ptr || !cam_ptr->IsInitialized()) {
        std::cout << "[Atenção] Câmera não inicializada para aplicar o crop." << std::endl;
        return;
    }

    try {
        // O nodeMap é o mapa de configurações dinâmico gerado a partir do XML interno da câmera, padrão GenICam.
        // Cada "nó" representa um registrador físico no HW da camera, exemplo: Width, Offset, etc.., e gerencia seus próprios limites e permissões.
        // Ele funciona como a interface segura e padronizada para ler e alterar o HW do device pelo código C++.
        Spinnaker::GenApi::INodeMap& nodeMap= cam_ptr->GetNodeMap();

        // Antes do cropped zera os Offsets para evitar estouro de limite físico:
        Spinnaker::GenApi::CIntegerPtr ptrOffsetX= nodeMap.GetNode("OffsetX");
        if (Spinnaker::GenApi::IsWritable(ptrOffsetX)) 
            ptrOffsetX->SetValue(0);

        Spinnaker::GenApi::CIntegerPtr ptrOffsetY= nodeMap.GetNode("OffsetY");
        if (Spinnaker::GenApi::IsWritable(ptrOffsetY)) 
            ptrOffsetY->SetValue(0);

        // Configura a resolução VGA:
        Spinnaker::GenApi::CIntegerPtr ptrWidth= nodeMap.GetNode("Width");
        if (Spinnaker::GenApi::IsWritable(ptrWidth)) 
            ptrWidth->SetValue(640);

        Spinnaker::GenApi::CIntegerPtr ptrHeight= nodeMap.GetNode("Height");
        if (Spinnaker::GenApi::IsWritable(ptrHeight)) 
            ptrHeight->SetValue(480);

        // Centraliza a imagem VGA 640x480 no sensor de 720x540:
        int offset_aux= (720-640)/2; // 40
        if (Spinnaker::GenApi::IsWritable(ptrOffsetX)) ptrOffsetX->SetValue(offset_aux);

        offset_aux= (540-480)/2; // 30
        if (Spinnaker::GenApi::IsWritable(ptrOffsetY)) ptrOffsetY->SetValue(offset_aux);

        // Guarad os valor atuaal da dimensão do sensor, seja cropado ou não cropado: 
        Width= ptrWidth->GetValue();
        Height= ptrHeight->GetValue();

        std::cout << " - Sensor cropped:\n ";
        std::cout << "  * Width  = " << Width << "\n"; 
        std::cout << "   * Height = " << Height << "\n";

    } catch (Spinnaker::Exception& e) {
        std::cout << "[Erro] Não foi possivel coroppar o sensor: " << e.what() << std::endl;
    }
}
