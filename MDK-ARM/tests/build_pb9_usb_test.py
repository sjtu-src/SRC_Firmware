"""Generate an isolated USB-only PB9 test project; normal sources stay untouched.

Disconnect the entire power-board cable before running the resulting firmware.
Use the normal robot firmware again before reconnecting the power board.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

MDK = Path(__file__).resolve().parents[1]
ROOT = MDK.parent
BUILD = MDK / 'tmp/pb9_usb_test'
OUTPUT = MDK / 'output/pb9_usb_test'
BUILD.mkdir(parents=True, exist_ok=True)
OUTPUT.mkdir(parents=True, exist_ok=True)

source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8-sig')
main_body = '''int main(void)
{
  /* BENCH ONLY: USB power, entire power-board cable disconnected.
   * No robot init, RTOS, wheels, kicking, or FG controller are started.
   * TIM11 retains the normal PB9 AF3 / PWM1 / low-polarity configuration.
   */
  HAL_Init();
  SystemClock_Config();
  MX_TIM11_Init();
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,
                        (__HAL_TIM_GET_AUTORELOAD(&htim11) + 1U) / 2U);
  for (;;)
  {
    __WFI();
  }
}'''
source, count = re.subn(r'(?m)^int main\(void\)\s*\n\{.*?^\}', main_body, source, count=1, flags=re.S)
assert count == 1
source = source.replace('static void MX_NVIC_Init(void);', '')
source, count = re.subn(r'(?m)^static void MX_NVIC_Init\(void\)\s*\n\{.*?^\}', '', source, count=1, flags=re.S)
assert count == 1
bench_main = BUILD / 'pb9_usb_main.c'
bench_main.write_text(source, encoding='utf-8', newline='\r\n')

def absolute(value):
    return str((MDK / value.replace('\\', '/')).resolve())

tree = ET.parse(MDK / 'SRC_Firmware.uvprojx')
project = tree.getroot()
# CMSIS core headers already come from the repository IncludePath. The normal
# project has a historical RTE-only CMSIS 4.5.0 selection; copying it triggers
# Pack Installer on project open, even though command-line builds can succeed.
for components in project.findall('./RTE/components'):
    for component in list(components):
        if component.get('Cclass') == 'CMSIS' and component.get('Cgroup') == 'CORE':
            components.remove(component)
for target_info in project.findall('./RTE//targetInfo'):
    target_info.set('name', 'PB9_USB_Test')
for groups in project.findall('./Targets/Target/Groups'):
    for group in list(groups):
        if group.findtext('GroupName') == '::CMSIS' and not group.findall('./Files/File'):
            groups.remove(group)
for node in project.iter('FilePath'):
    path = Path(absolute(node.text))
    node.text = str(bench_main) if path == (ROOT/'Core/Src/main.c').resolve() else str(path)
for node in project.iter('FileName'):
    if node.text == 'main.c': node.text = bench_main.name
for node in project.iter('IncludePath'):
    if node.text:
        node.text = ';'.join(absolute(p) for p in node.text.split(';') if p)
for tag in ('ScatterFile', 'RegisterFilePath', 'DBRegisterFilePath'):
    for node in project.iter(tag):
        if node.text: node.text = absolute(node.text)
for node in project.iter('TargetName'): node.text = 'PB9_USB_Test'
for node in project.iter('OutputDirectory'): node.text = str(OUTPUT) + '\\'
for node in project.iter('ListingPath'): node.text = str(BUILD) + '\\'
for node in project.iter('OutputName'): node.text = 'PB9_USB_Test'
for node in project.iter('CreateBatchFile'): node.text = '0'
project_path = BUILD / 'PB9_USB_Test.uvprojx'
tree.write(project_path, encoding='utf-8', xml_declaration=True)
print(project_path)
print('Expected output: 25 kHz, 50% active-low duty, continuous PB9 PWM.')
print('Normal main.c and normal project are unchanged.')
