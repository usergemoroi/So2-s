#include <list>
#include <vector>
#include <string.h>
#include <pthread.h>
#include <thread>
#include <cstring>
#include <jni.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <dlfcn.h>
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include "Includes/Logger.h"
#include "Includes/obfuscate.h"
#include "Includes/Utils.h"

#include "imgui.h"
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui_internal.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_android.h"
#define targetLibName OBFUSCATE("libil2cpp.so")

#include "Vector3.h"


int glHeight, glWidth;
bool setup;


#include <libgen.h>
#include <fcntl.h>
#include <inttypes.h>
#include <jni.h>
#include <unistd.h>
#include <sys/mman.h>
#include <dirent.h>
#include <map>



#include "Zygisk/zygisk.h"

#include "font/font.h"
#include "font/PixelFont.h"
#include "font/SkeetFont.h"
#include "font/icons.h"
#include "font/weapon.h"
#include "Xhook/xhook.h"

uintptr_t il2cpp_base;

ImFont*weapon;
ImFont*SkeetNormal;
ImFont*SkeetSmall;
void SetupImgui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)glWidth, (float)glHeight);
    ImGui_ImplOpenGL3_Init("#version 100");
	SkeetNormal = io.Fonts->AddFontFromMemoryTTF(&PixelFont, sizeof PixelFont, 21, NULL, io.Fonts->GetGlyphRangesCyrillic());
	SkeetSmall = io.Fonts->AddFontFromMemoryTTF(&PixelFont, sizeof PixelFont, 20, NULL, io.Fonts->GetGlyphRangesCyrillic());
	weapon = io.Fonts->AddFontFromMemoryTTF(&weapon1, sizeof weapon1, 30.0f, NULL);
    ImFontConfig font_cfg;
    font_cfg.SizePixels = 22.0f;
    io.Fonts->AddFontDefault(&font_cfg);
    ImGui::GetStyle().ScaleAllSizes(2.3f);
}


template <typename T>
struct monoArray {
    void *klass;
    void *monitor;
    void *bounds;
    int32_t capacity;
    T m_Items[0];

    [[maybe_unused]] int getCapacity() {
        if (!this) return 0;
        return capacity;
    }

    T *getPointer() {
        if (!this) return nullptr;
        return m_Items;
    }

    std::vector<T> toCPPlist() {
        if (!this) return {};
        std::vector<T> ret;
        for (int i = 0; i < capacity; i++)
            ret.push_back(m_Items[i]);
        return std::move(ret);
    }

    bool copyFrom(const std::vector<T> &vec) {
        if (!this) return false;
        return copyFrom((T *) vec.data(), (int) vec.size());
    }

    [[maybe_unused]] bool copyFrom(T *arr, int size) {
        if (!this) return false;
        if (size < capacity)
            return false;
        memcpy(m_Items, arr, size * sizeof(T));
        return true;
    }

    [[maybe_unused]] void copyTo(T *arr) {
        if (!this || !CheckObj(m_Items)) return;
        memcpy(arr, m_Items, sizeof(T) * capacity);
    }

    T operator[](int index) {
        if (getCapacity() < index) return {};
        return m_Items[index];
    }

    T at(int index) {
        if (!this || getCapacity() <= index || empty()) return {};
        return m_Items[index];
    }

    bool empty() {
        if (!this) return false;
        return getCapacity() <= 0;
    }

    monoArray<T> *Create(int size) {
        monoArray<T> *monoArr = (monoArray<T> *) malloc(sizeof(monoArray) + sizeof(T) * size);
        monoArr->capacity = size;
        return monoArr;
    }

    static monoArray<T> *Create(const std::vector<T> &vec) {
        return Create(vec.data(), vec.size());
    }

    static monoArray<T> *Create(T *arr, int size) {
        monoArray<T> *monoArr = Create(size);
        monoArr->copyFrom(arr, size);
        return monoArr;
    }
};




template<typename TKey, typename TValue>
struct monoDictionary {
    struct Entry {
        [[maybe_unused]] int hashCode, next;
        TKey key;
        TValue value;
    };
    void *klass;
    void *monitor;
    [[maybe_unused]] monoArray<int> *buckets;
    monoArray<Entry> *entries;
    int count;
    int version;
    [[maybe_unused]] int freeList;
    [[maybe_unused]] int freeCount;
    void *compare;
    monoArray<TKey> *keys;
    monoArray<TValue> *values;
    [[maybe_unused]] void *syncRoot;
    std::vector<TKey> getKeys() {
        std::vector<TKey> ret;
        auto lst = entries->toCPPlist();
        for(auto enter : lst) ret.push_back(enter.key);
        return std::move(ret);
    }
    std::vector<TValue> getValues() {
        std::vector<TValue> ret;
        auto lst = entries->toCPPlist();
        for(auto enter : lst) ret.push_back(enter.value);
        return std::move(ret);
    }
    std::map<TKey, TValue> toMap() {
        std::map<TKey, TValue> ret;
        for (auto it = (Entry*)&entries->m_Items; it != ((Entry*)&entries->m_Items + count); ++it) ret.emplace(std::make_pair(it->key, it->value));
        return std::move(ret);
    }
    int getSize() { return count; }
    [[maybe_unused]] int getVersion() { return version; }
    bool TryGet(TKey key, TValue &value) { }
    [[maybe_unused]] void Add(TKey key, TValue value) { }
    [[maybe_unused]] void Insert(TKey key, TValue value) { }
    [[maybe_unused]] bool Remove(TKey key) { }
    [[maybe_unused]] bool ContainsKeyString(const char* key_to_find) {
        auto valuesVector = getValues();
        for (int j = 0; j > valuesVector.size(); ++j) {
            if (!strcmp(valuesVector[j]->c_str(), key_to_find)) {
                return true;
            }
        }
        return false;
    }
    [[maybe_unused]] bool ContainsValue(TValue value) { }
    TValue Get(TKey key) {
        TValue ret;
        if (TryGet(key, ret))
            return ret;
        return {};
    }
    TValue operator [](TKey key)  { return Get(key); }
};

long mono_address = 0;
using namespace std;std::string utf16le_to_utf8(const std::u16string &u16str) {
    if (u16str.empty()) {
        return std::string();
    }
    const char16_t *p = u16str.data();
    std::u16string::size_type len = u16str.length();
    if (p[0] == 0xFEFF) {
        p += 1;
        len -= 1;
    }
    std::string u8str;
    u8str.reserve(len * 3);
    char16_t u16char;
    for (std::u16string::size_type i = 0; i < len; ++i) {
        u16char = p[i];
        if (u16char < 0x0080) {
            u8str.push_back((char) (u16char & 0x00FF));
            continue;
        }
        if (u16char >= 0x0080 && u16char <= 0x07FF) {
            u8str.push_back((char) (((u16char >> 6) & 0x1F) | 0xC0));
            u8str.push_back((char) ((u16char & 0x3F) | 0x80));
            continue;
        }
        if (u16char >= 0xD800 && u16char <= 0xDBFF) {
            uint32_t highSur = u16char;
            uint32_t lowSur = p[++i];
            uint32_t codePoint = highSur - 0xD800;
            codePoint <<= 10;
            codePoint |= lowSur - 0xDC00;
            codePoint += 0x10000;
            u8str.push_back((char) ((codePoint >> 18) | 0xF0));
            u8str.push_back((char) (((codePoint >> 12) & 0x3F) | 0x80));
            u8str.push_back((char) (((codePoint >> 06) & 0x3F) | 0x80));
            u8str.push_back((char) ((codePoint & 0x3F) | 0x80));
            continue;
        }
        {
            u8str.push_back((char) (((u16char >> 12) & 0x0F) | 0xE0));
            u8str.push_back((char) (((u16char >> 6) & 0x3F) | 0x80));
            u8str.push_back((char) ((u16char & 0x3F) | 0x80));
            continue;
        }
    }
    return u8str;
}
typedef struct _monoString {
    void *klass;
    void *monitor;
    int length;
    const char *toChars(){
        u16string ss((char16_t *) getChars(), 0, getLength()); string str = utf16le_to_utf8(ss); return str.c_str();
    } char chars[0]; char *getChars() { return chars; }
    int getLength() {
        return length;
    }
    std::string get_string() {
        return std::string(toChars());
    }
}monoString;


uintptr_t GetBaseAddress(const char *name) {
    uintptr_t base = 0;
    char line[512];

    FILE *f = fopen(OBFUSCATE("/proc/self/maps"), OBFUSCATE("r"));

    if (!f) {
        return 0;
    }
    while (fgets(line, sizeof line, f)) {
        uintptr_t tmpBase;
        char tmpName[256];
        if (sscanf(line, "%" PRIXPTR "-%*" PRIXPTR "%*s %*s %*s %*s %s", &tmpBase, tmpName) > 0) {
            if (!strcmp(basename(tmpName), name)) {
                base = tmpBase;
                break;
            }
        }
    }
    fclose(f);
    return base;
}

std::vector<uintptr_t>get_player_list()
{
	
	std::vector<uintptr_t>null_vec;
	
	uintptr_t playermanager_getinstance = *(uintptr_t*)(il2cpp_base+0x4BED7A8);
	uintptr_t playermanager_il2cppclass = *(uintptr_t*)(playermanager_getinstance+0x20);
	uintptr_t playermanager_staticfields = *(uintptr_t*)(playermanager_il2cppclass+0xB8);
	uintptr_t playermanager_instance = *(uintptr_t*)(playermanager_staticfields+0x0);
	
	if (playermanager_instance) 
	{
		auto entities = *(monoDictionary<int,uintptr_t>**)(playermanager_instance+0x28);
		return entities->getValues();
	}
	else
		return null_vec;
}

uintptr_t get_local_player()
{
	uintptr_t playermanager_getinstance = *(uintptr_t*)(il2cpp_base+0x4BED7A8);
	uintptr_t playermanager_il2cppclass = *(uintptr_t*)(playermanager_getinstance+0x20);
	uintptr_t playermanager_staticfields = *(uintptr_t*)(playermanager_il2cppclass+0xB8);
	uintptr_t playermanager_instance = *(uintptr_t*)(playermanager_staticfields+0x0);
	auto local_player = *(uintptr_t*)(playermanager_instance+0x80);
	return local_player;
}

namespace colors {
	namespace esp {
		namespace grenade {
			float circlecolor[] = {0,0,0,1};
			float linecolor[] = {1,1,1,1};
		}
		namespace player {
			float HpColor[] = {1,1,1};
			float fillColor[]={1,1,1};
			float fill_value = 150;
			float boxColor[] = {1,1,1};
			float armorColor[] = {1,1,1};
			float skeletonColor[] = {1,1,1};
		}
	}
}
float hpcolor1[] = { 1,0,0 };
float hpcolor2[] = { 1,0,0 };
float armorcolor1[] = { 1,0,0 };
float armorcolor2[] = { 1,0,0 };

namespace esp{
	namespace grenade{
		bool enable;
		bool circle;
		bool name;
		bool outoffov;
	}
	namespace player {
		bool enable;
		bool box;
		bool fill;
		bool fooots;
		bool health;
		bool name;
		bool dist;
		bool armor;
		bool line;
		const char* hptype[] = {"Default","Gradient"};
		const char* armortypes[] = {"Default","Gradient"};
		int healthtype;
		int armortype;
		bool skeleton;
		bool money;
		float glow;
		bool weapon;
	}
	namespace weapon{
		bool enable;
	}
}




void DrawRectGlow(ImVec2 start, ImVec2 end, ImVec4 col,int gsize) {
    for (int i = 0; i < gsize; i++) {
        ImGui::GetBackgroundDrawList()->AddRectFilled(ImVec2(start.x - i, start.y - i), ImVec2(end.x + i, end.y + i), ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, col.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))));
    }
    ImGui::GetBackgroundDrawList()->AddRectFilled(start, end, ImGui::GetColorU32(col));
}

void DrawRectGlow1(ImVec2 start, ImVec2 end, ImVec4 col, ImVec4 col1, int gsize) {
    for (int i = 0; i < gsize; i++) {
        ImGui::GetBackgroundDrawList()->AddRectFilledMultiColor(ImVec2(start.x - i, start.y - i), ImVec2(end.x + i, end.y + i), ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, col.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, col.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col1.x, col1.y, col1.z, col1.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col1.x, col1.y, col1.z, col1.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))));
    }
    ImGui::GetBackgroundDrawList()->AddRectFilledMultiColor(start, end, ImGui::GetColorU32(col), ImGui::GetColorU32(col), ImGui::GetColorU32(col1), ImGui::GetColorU32(col1));
}
void DrawRectGlow2(ImVec2 start, ImVec2 end, ImVec4 col, ImVec4 col1, int gsize) {
    for (int i = 0; i < gsize; i++) {
        ImGui::GetBackgroundDrawList()->AddRectFilledMultiColor(ImVec2(start.x - i, start.y - i), ImVec2(end.x + i, end.y + i), ImGui::GetColorU32(ImVec4(col1.x, col1.y, col1.z, col1.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, col.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, col.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))), ImGui::GetColorU32(ImVec4(col1.x, col1.y, col1.z, col1.w * (1.0f / (float)gsize) * (((float)(gsize - i)) / (float)gsize))));
    }
    ImGui::GetBackgroundDrawList()->AddRectFilledMultiColor(start, end, ImGui::GetColorU32(col1), ImGui::GetColorU32(col), ImGui::GetColorU32(col), ImGui::GetColorU32(col1));
}

std::map<int, std::string> weaptofont = {
        {1, OBFUSCATE("b")},
        {2, OBFUSCATE("b")},
        {11, OBFUSCATE("A")},
        {12, OBFUSCATE("B")},
        {13, OBFUSCATE("C")},
        {16, OBFUSCATE("E")},
        {17, OBFUSCATE("F")},
        {49, OBFUSCATE("Q")},
        {15, OBFUSCATE("G")},
        {32, OBFUSCATE("M")},
        {34, OBFUSCATE("O")},
        {35, OBFUSCATE("P")},
        {36, OBFUSCATE("N")},
        {37, OBFUSCATE("L")},
        {43, OBFUSCATE("U")},
        {44, OBFUSCATE("S")},
        {45, OBFUSCATE("V")},
        {46, OBFUSCATE("T")},
        {47, OBFUSCATE("W")},
        {48, OBFUSCATE("R")},
        {51, OBFUSCATE("Y")},
        {52, OBFUSCATE("X")},
        {62, OBFUSCATE("I")},
        {70, OBFUSCATE("b")},
        {71, OBFUSCATE("c")},
        {72, OBFUSCATE("d")},
        {73, OBFUSCATE("e")},
        {75, OBFUSCATE("f")},
        {77, OBFUSCATE("g")},
        {78, OBFUSCATE("i")},
        {91, OBFUSCATE("p")},
        {92, OBFUSCATE("n")},
        {93, OBFUSCATE("o")},
        {100, OBFUSCATE("a")},
        {69, OBFUSCATE("I")},
        {74, OBFUSCATE("b")},
        {99, OBFUSCATE("p")},
        {79, OBFUSCATE("h")},
        {42, OBFUSCATE("b")},
        {81, OBFUSCATE("l")},
        {82, OBFUSCATE("k")},
        {53, OBFUSCATE("Z")},
        {63, OBFUSCATE("H")}
};

Vector3 (*get_position)(uintptr_t);
Vector3 (*Wts)(uintptr_t,Vector3);

uintptr_t (*get_transform)(uintptr_t);
uintptr_t (*get_camera)();

int (*GetMoney)(uintptr_t);
int (*GetArmor)(uintptr_t);
int (*GetHealth)(uintptr_t);

monoString* Getname(uintptr_t player){
	return *(monoString**)(player+0x20);
}

uintptr_t get_photon(uintptr_t player)
{
	return *(uintptr_t*)(player+0x130);
}


int get_3D_Distance(int Self_x, int Self_y, int Self_z, int Object_x, int Object_y, int Object_z)
{
    int x, y, z;
    x = Self_x - Object_x;
    y = Self_y - Object_y;
    z = Self_z - Object_z;
    return (int)(sqrt(x * x + y * y + z * z));
}
monoString*GetPlayerWeapon(uintptr_t a) {
    auto w1 = *(uintptr_t *) (a + 0x68);
    if (w1) {
        auto w2 = *(uintptr_t *) (w1 + 0x98);
        if (w2) {
            auto w3 = *(uintptr_t *) (w2 + 0xA0);
            if (w3) {
                auto w4 = *(monoString**) (w3 + 0x20);
                if (w4) return w4;
            }
        }
    }
}
int get_weaponid(uintptr_t a) {
    auto w1 = *(uintptr_t *) (a + 0x68);
    if (w1) {
        auto w2 = *(uintptr_t *) (w1 + 0x98);
        if (w2) {
            auto w3 = *(uintptr_t *) (w2 + 0xA0);
            if (w3) {
                auto w4 = *(int *) (w3 + 0x18);
                if (w4) return w4;
            }
        }
    }
}

void render_menu(){
	ImGui::Begin("zopa");
	
		ImGui::Checkbox(OBFUSCATE("Esp"),&esp::player::enable);
		ImGui::Checkbox(OBFUSCATE("Box"),&esp::player::box);
		ImGui::Checkbox(OBFUSCATE("Fill"),&esp::player::fill);
		ImGui::Checkbox(OBFUSCATE("Health"),&esp::player::health);
		ImGui::Checkbox(OBFUSCATE("Name"),&esp::player::name);
		ImGui::Checkbox(OBFUSCATE("Distance"),&esp::player::dist);
		ImGui::Checkbox(OBFUSCATE("Armor"),&esp::player::armor);
		//ImGui::Checkbox(OBFUSCATE("Skeleton"),&esp::player::skeleton);
		ImGui::Checkbox(OBFUSCATE("Money"),&esp::player::money);
    	ImGui::Checkbox(OBFUSCATE("Weapon"),&esp::player::weapon);
		ImGui::ColorEdit3(OBFUSCATE("BoxColor"),colors::esp::player::boxColor,ImGuiColorEditFlags_NoInputs);
		ImGui::Combo(OBFUSCATE("Armor Type"), &esp::player::armortype, esp::player::armortypes, IM_ARRAYSIZE(esp::player::armortypes));
		switch (esp::player::armortype){
			case 0:
				ImGui::ColorEdit3(OBFUSCATE("Armor Color"),colors::esp::player::armorColor,ImGuiColorEditFlags_NoInputs);
			break;
			case 1:
				ImGui::ColorEdit3(OBFUSCATE("Armor Color1"), armorcolor2,ImGuiColorEditFlags_NoInputs);
				ImGui::ColorEdit3(OBFUSCATE("Armot Color2"), armorcolor1,ImGuiColorEditFlags_NoInputs);
			break;
		}
    	ImGui::Combo(OBFUSCATE("Health Type"), &esp::player::healthtype, esp::player::hptype, IM_ARRAYSIZE(esp::player::hptype));
		switch (esp::player::healthtype){
			case 0:
				ImGui::ColorEdit3(OBFUSCATE("Health Color"),colors::esp::player::HpColor,ImGuiColorEditFlags_NoInputs);
			break;
			case 1:
				ImGui::ColorEdit3(OBFUSCATE("Hp Color1"), hpcolor1,ImGuiColorEditFlags_NoInputs);
				ImGui::ColorEdit3(OBFUSCATE("Hp Color2"), hpcolor2,ImGuiColorEditFlags_NoInputs);
			break;
		}
    	ImGui::SliderFloat(OBFUSCATE("Glow Size"),&esp::player::glow,0,30);
		//ImGui::ColorEdit4(OBFUSCATE("Skeleton Color"),colors::esp::player::skeletonColor,ImGuiColorEditFlags_NoInputs);
		ImGui::SliderFloat(OBFUSCATE("Fill Value"),&colors::esp::player::fill_value,0,255);
	
	if (esp::player::enable)
	{
		
			for (const auto & player : get_player_list())
			{
				
				if(player==NULL) continue;
				
				if (get_local_player()==NULL) continue;
				
				auto team = *(uint8_t*)(player+0x59);
				
				auto local_team = *(uint8_t*)(get_local_player()+0x59);
				
				if(team == local_team) continue;
				
					auto pl = player;
					Vector3 posup = get_position(get_transform(pl))+Vector3(0,1.9f,0);
					Vector3 posdown = get_position(get_transform(pl));
					Vector3 posuponscr = Wts(get_camera(),posup);
					Vector3 posdownonscr = Wts(get_camera(),posdown);
					float height = posuponscr.y-posdownonscr.y;
					float width = height / 2;
					auto render = ImGui::GetBackgroundDrawList();
					ImVec2 DrawTo = ImVec2(posuponscr.x, (glHeight - posuponscr.y));
				
					auto weap = GetPlayerWeapon(pl);
                	std::string weaps = "null";
                	if (weap) weaps = weap->get_string();
                	std::transform(weaps.begin(), weaps.end(), weaps.begin(), ::tolower);
					std::string wpn = weaps;
				
				
					if(posuponscr.z > 1.0f){
					
						if(esp::player::weapon){
							int wid = get_weaponid(pl);
							std::string weapng = weaptofont[wid];	
							int h = GetArmor(get_photon(pl));
							
							if(!esp::player::armor||h==0) {
								
								ImGui::PushFont(SkeetSmall);
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)-1,(glHeight-posdownonscr.y)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)-1,(glHeight-posdownonscr.y)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)+1,(glHeight-posdownonscr.y)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)+1,(glHeight-posdownonscr.y)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
					    		render->AddText(ImVec2(posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2,glHeight-posdownonscr.y), ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, 1)), wpn.c_str());
								ImGui::PopFont();
							
								ImGui::PushFont(weapon);
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2),(glHeight-posdownonscr.y+ImGui::CalcTextSize(weapng.c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)-1,(glHeight-posdownonscr.y+ImGui::CalcTextSize(weapng.c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)+1,(glHeight-posdownonscr.y+ImGui::CalcTextSize(weapng.c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)+1,(glHeight-posdownonscr.y+ImGui::CalcTextSize(weapng.c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
					    		render->AddText(ImVec2(posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2,glHeight-posdownonscr.y+ImGui::CalcTextSize(weapng.c_str()).y/2), ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, 1)), weapng.c_str());
								ImGui::PopFont();
							
							} else {
								
								ImGui::PushFont(SkeetSmall);
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)-1,(glHeight-posdownonscr.y)+8), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)-1,(glHeight-posdownonscr.y)+10), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)+1,(glHeight-posdownonscr.y)+8), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2)+1,(glHeight-posdownonscr.y)+10), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), wpn.c_str());
					    		render->AddText(ImVec2(posuponscr.x-ImGui::CalcTextSize(wpn.c_str()).x/2,glHeight-posdownonscr.y+9), ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, 1)), wpn.c_str());
								ImGui::PopFont();
							
								ImGui::PushFont(weapon);
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)-1,(glHeight-posdownonscr.y+9+ImGui::CalcTextSize(weapng.c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
								render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)-1,(glHeight-posdownonscr.y+9+ImGui::CalcTextSize(weapng.c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)+1,(glHeight-posdownonscr.y+9+ImGui::CalcTextSize(weapng.c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
		   						render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2)+1,(glHeight-posdownonscr.y+9+ImGui::CalcTextSize(weapng.c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), weapng.c_str());
					    		render->AddText(ImVec2(posuponscr.x-ImGui::CalcTextSize(weapng.c_str()).x/2,glHeight-posdownonscr.y+9+ImGui::CalcTextSize(weapng.c_str()).y/2), ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, 1)), weapng.c_str());
								ImGui::PopFont();
							
							}
						}
											
						if(esp::player::fill)
							render->AddRectFilled(ImVec2(posuponscr.x - (width/2),glHeight-posuponscr.y),ImVec2(posuponscr.x+(width/2),glHeight-posuponscr.y+height),ImColor(colors::esp::player::boxColor[0],colors::esp::player::boxColor[1],colors::esp::player::boxColor[2],colors::esp::player::fill_value/255));
							
						if(esp::player::box){
								
							render->AddRect(ImVec2(posuponscr.x - (width/2),glHeight-posuponscr.y),ImVec2(posuponscr.x+(width/2),glHeight-posuponscr.y+height),ImColor(0,0,0),0,0,3.5f);
							render->AddRect(ImVec2(posuponscr.x - (width/2),glHeight-posuponscr.y),ImVec2(posuponscr.x+(width/2),glHeight-posuponscr.y+height),ImColor(colors::esp::player::boxColor[0],colors::esp::player::boxColor[1],colors::esp::player::boxColor[2]),0,0,1.5f);
								
						}
							
						if(esp::player::health){
						
							float health = GetHealth(get_photon(pl));
							int h = ((int) health);
                    		int x = DrawTo.x - width/2 - 10;
                   			int y = DrawTo.y;
                    		auto Render = render;
							
							if(esp::player::healthtype==0) {
								Render->AddRectFilled(ImVec2(x, y - 1),ImVec2(x + 6, y + 1 + height),ImColor(0, 0, 0, 200));
								DrawRectGlow(ImVec2(x + 1,y +height -(height *(health /100))),ImVec2(x + 5,y +height),ImVec4(colors::esp::player::HpColor[0],colors::esp::player::HpColor[1],colors::esp::player::HpColor[2],1),(int)esp::player::glow);
							}
							if(esp::player::healthtype==1) {
								Render->AddRectFilled(ImVec2(x, y - 1),ImVec2(x + 6, y + 1 + height),ImColor(0, 0, 0, 200));
								DrawRectGlow1(ImVec2(x + 1,y +height -(height *(health/100))),ImVec2(x + 5,y +height), ImVec4(hpcolor1[0], hpcolor1[1], hpcolor1[2], 1.0f), ImVec4(hpcolor2[0], hpcolor2[1], hpcolor2[2], 1.0f),(int)esp::player::glow);
							}
							if(h <100){
								ImGui::PushFont(SkeetSmall);
				   				render->AddText(ImVec2((x+3)-ImGui::CalcTextSize(std::to_string(h).c_str()).x/2,y-9 +height -(height *(health /100))), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());		  
								render->AddText(ImVec2((x+3)-ImGui::CalcTextSize(std::to_string(h).c_str()).x/2,y -7+height -(height *(health /100))), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());		  
								render->AddText(ImVec2((x+5)-ImGui::CalcTextSize(std::to_string(h).c_str()).x/2,y-9 +height -(height *(health /100))), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());		  
								render->AddText(ImVec2((x+5)-ImGui::CalcTextSize(std::to_string(h).c_str()).x/2,y -7+height -(height *(health /100))), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());		
								render->AddText(ImVec2((x+4)-ImGui::CalcTextSize(std::to_string(h).c_str()).x/2,y-8+height -(height *(health /100))), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), std::to_string(h).c_str());
								ImGui::PopFont();
							}
						}
											if(esp::player::skeleton){
												//DrawSkeleton(pl,ImColor(colors::esp::player::skeletonColor[0],colors::esp::player::skeletonColor[1],colors::esp::player::skeletonColor[2],1.0f));
											}
						if(esp::player::name){
							monoString *isPlayerName = Getname(get_photon(pl));
							ImGui::PushFont(SkeetNormal);					
							render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(isPlayerName->get_string().c_str()).x/2)+1.8f,(glHeight-posuponscr.y- 20)+1.8f), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), isPlayerName->get_string().c_str());
							render->AddText(ImVec2((posuponscr.x-ImGui::CalcTextSize(isPlayerName->get_string().c_str()).x/2),glHeight-posuponscr.y- 20), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), isPlayerName->get_string().c_str());
							ImGui::PopFont();
						}
						if(esp::player::money){
							int mon = GetMoney(get_photon(pl));
							ImGui::PushFont(SkeetSmall);
				   			render->AddText(ImVec2((posuponscr.x+5+width/2)-1,(glHeight-posuponscr.y+12)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(mon)+"$").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)-1,(glHeight-posuponscr.y+12)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(mon)+"$").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)+1,(glHeight-posuponscr.y+12)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(mon)+"$").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)+1,(glHeight-posuponscr.y+12)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(mon)+"$").c_str());
							render->AddText(ImVec2(posuponscr.x+5+width/2,glHeight-posuponscr.y+12), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), (std::to_string(mon)+"$").c_str());
							ImGui::PopFont();
						}
						if(esp::player::dist){
							Vector3 mypos = get_position(get_transform(get_camera()));
				   			int DistanceTo=get_3D_Distance(mypos.x, mypos.y, mypos.z, posup.x, posup.y, posup.z);
							ImGui::PushFont(SkeetSmall);
				   			render->AddText(ImVec2((posuponscr.x+5+width/2)-1,(glHeight-posuponscr.y-2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(DistanceTo)+"M").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)-1,(glHeight-posuponscr.y-2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(DistanceTo)+"M").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)+1,(glHeight-posuponscr.y-2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(DistanceTo)+"M").c_str());
							render->AddText(ImVec2((posuponscr.x+5+width/2)+1,(glHeight-posuponscr.y-2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), (std::to_string(DistanceTo)+"M").c_str());
							render->AddText(ImVec2(posuponscr.x+5+width/2,glHeight-posuponscr.y-2), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), (std::to_string(DistanceTo)+"M").c_str());
							ImGui::PopFont();
						}
						if(esp::player::armor){
							int h = GetArmor(get_photon(pl));
							if(h>0){
								if(esp::player::armortype==0){
									render->AddRectFilled(ImVec2(posuponscr.x-1 - width / 2, glHeight - posdownonscr.y+3), ImVec2(posuponscr.x+1 + width / 2, glHeight - posdownonscr.y+9), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1.0f)));
									DrawRectGlow(ImVec2(posuponscr.x - width / 2, glHeight - posdownonscr.y + 4), ImVec2((posuponscr.x - float(width / 2) + float(width * (h / 100.f))), glHeight - posdownonscr.y + 8), ImVec4(colors::esp::player::armorColor[0],colors::esp::player::armorColor[1],colors::esp::player::armorColor[2],1), (int)esp::player::glow);
									if(h<100){
										ImGui::PushFont(SkeetSmall);
				   						render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f)))-1,(glHeight - posdownonscr.y + 7-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x-11 - float(width / 2) + float(width * (h / 100.f)))-1,(glHeight - posdownonscr.y + 7-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x-11 - float(width / 2) + float(width * (h / 100.f)))+1,(glHeight - posdownonscr.y + 7-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x-11 - float(width / 2) + float(width * (h / 100.f)))+1,(glHeight - posdownonscr.y + 7-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)),std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f))),glHeight - posdownonscr.y + 7-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), std::to_string(h).c_str());
										ImGui::PopFont();
									}
								}
								if(esp::player::armortype == 1){
									render->AddRectFilled(ImVec2(posuponscr.x-1 - width / 2, glHeight - posdownonscr.y+3), ImVec2(posuponscr.x+1 + width / 2, glHeight - posdownonscr.y+9), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1.0f)));
        							DrawRectGlow2(ImVec2(posuponscr.x - width / 2, glHeight - posdownonscr.y + 4), ImVec2((posuponscr.x - float(width / 2) + float(width * (h / 100.f))), glHeight - posdownonscr.y + 8),  ImVec4(armorcolor1[0], armorcolor1[1], armorcolor1[2], 1.0f), ImVec4(armorcolor2[0], armorcolor2[1], armorcolor2[2], 1.0f),(int)esp::player::glow);
									if(h<100){
										ImGui::PushFont(SkeetSmall);
				   						render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f)))-1,(glHeight - posdownonscr.y + 6-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f)))-1,(glHeight - posdownonscr.y + 6-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f)))+1,(glHeight - posdownonscr.y + 6-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)+1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)), std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f)))+1,(glHeight - posdownonscr.y + 6-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2)-1), ImGui::ColorConvertFloat4ToU32(ImVec4(0,0,0, 1)),std::to_string(h).c_str());
										render->AddText(ImVec2((posuponscr.x -11- float(width / 2) + float(width * (h / 100.f))),glHeight - posdownonscr.y + 6-ImGui::CalcTextSize(std::to_string(h).c_str()).y/2), ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1, 1)), std::to_string(h).c_str());
										ImGui::PopFont();
									}
								}
							}
						}
					}
		
			}
			//ImGui::Text(std::to_string(entities->getSize()).c_str());
		
	}
	ImGui::End();
}




bool (*old_eglSwapBuffers)(EGLDisplay dpy, EGLSurface surface);
bool hook_eglSwapBuffers(EGLDisplay dpy, EGLSurface surface) {
    eglQuerySurface(dpy, surface, EGL_WIDTH, &glWidth);
    eglQuerySurface(dpy, surface, EGL_HEIGHT, &glHeight);

    if (!setup) {
        SetupImgui();
        setup = true;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
	
    render_menu();
 
    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


    return old_eglSwapBuffers(dpy, surface);
}


void (*old_input)(void *event, void *exAb, void *exAc);
void hook_input(void *event, void *exAb, void *exAc) {
    old_input(event, exAb, exAc);
    ImGui_ImplAndroid_HandleInputEvent((AInputEvent *) event);
    return;
}



void *hack_threads(const char *) {

    do {
        sleep(1);
    } while (!isLibraryLoaded("libil2cpp.so"));
	
	il2cpp_base = GetBaseAddress("libil2cpp.so");
	
	get_position = (Vector3(*)(uintptr_t))(il2cpp_base+(0x45B4B68)); //есть
	Wts = (Vector3(*)(uintptr_t,Vector3))(il2cpp_base+(0x457BA18)); //есть
	get_transform = (uintptr_t(*)(uintptr_t))(il2cpp_base+(0x45A8990)); //есть
	get_camera = (uintptr_t(*)())(il2cpp_base+(0x457C0C4)); //есть
	GetHealth = (int(*)(uintptr_t))(il2cpp_base+0x23F70A4);//есть
	GetArmor = (int(*)(uintptr_t))(il2cpp_base+0x23F5DE0);// есть
	GetMoney = (int(*)(uintptr_t))(il2cpp_base+0x231E2E4);
	
	xhook_register(OBFUSCATE(".*\\.so$"), OBFUSCATE("_ZN7android13InputConsumer21initializeMotionEventEPNS_11MotionEventEPKNS_12InputMessageE"), (void *) hook_input, (void **) &old_input);
	xhook_register(OBFUSCATE(".*\\.so$"), OBFUSCATE("eglSwapBuffers"), (void*)hook_eglSwapBuffers, (void**)&old_eglSwapBuffers);
	xhook_refresh(0);
	
	
    pthread_exit(nullptr);
    return nullptr;


}

#include "zygiskm.h"
REGISTER_ZYGISK_MODULE(NewQQ)
