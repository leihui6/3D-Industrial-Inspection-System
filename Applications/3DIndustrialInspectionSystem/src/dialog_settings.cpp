#include "dialog_settings.h"
#include "ui_dialog_settings.h"

dialog_settings::dialog_settings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dialog_settings)
{
    ui->setupUi(this);

}

void dialog_settings::set_configuration(std::map<std::string, std::string> & str_str_map)
{
    m_dialog_str_str_map = str_str_map;

    QString default_config_path(m_dialog_str_str_map["configuration"].data());
    this->ui->lineEdit_configuration->setText(default_config_path);

    //on_btn_reload_clicked();
}

dialog_settings::~dialog_settings()
{
    delete ui;
}

void dialog_settings::on_btn_reload_clicked()
{
    ui->lineEdit_reading_data->clear();
    ui->lineEdit_reference_data->clear();
    ui->lineEdit_output_folder->clear();
    ui->lineEdit_icp_configuration->clear();
    ui->lineEdit_4pcs_configuration->clear();
    ui->lineEdit_measurement_filename->clear();

    m_ui_configuration = ui->lineEdit_configuration->text().toStdString();

    m_dialog_str_str_map.clear();
    if (!read_file_as_map(m_ui_configuration, m_dialog_str_str_map))
        return;

    ui->lineEdit_reading_data->setText(QString(m_dialog_str_str_map["reading_data"].data()));
    ui->lineEdit_reference_data->setText(QString(m_dialog_str_str_map["reference_data"].data()));
    ui->lineEdit_output_folder->setText(QString(m_dialog_str_str_map["output_folder"].data()));
    ui->lineEdit_icp_configuration->setText(QString(m_dialog_str_str_map["icp_configuration"].data()));
    ui->lineEdit_4pcs_configuration->setText(QString(m_dialog_str_str_map["4pcs_configuration"].data()));
    ui->lineEdit_measurement_filename->setText(QString(m_dialog_str_str_map["measurement_pairs"].data()));
}
