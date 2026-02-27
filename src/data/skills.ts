// Skill data configuration file
// Used to manage data for the skill display page

export interface Skill {
	id: string;
	name: string;
	description: string;
	icon: string; // Iconify icon name
	category: "embedded" | "Programming" | "视觉" | "tools" | "other";
	level: "beginner" | "intermediate" | "advanced" | "expert";
	experience: {
		years: number;
		months: number;
	};
	projects?: string[]; // Related project IDs
	certifications?: string[];
	color?: string; // Skill card theme color
}

export const skillsData: Skill[] = [
	// Frontend Skills
	{
		id: "STM32",
		name: "STM32",
		description:
			"STM32是意法半导体推出的一系列基于ARM Cortex-M内核的微控制器,广泛应用于嵌入式系统开发中。",
		icon: "simple-icons:stmicroelectronics",
		category: "embedded",
		level: "beginner",
		experience: { years: 0, months: 5 },
		projects: ["system-design"

		],
		color: "#03234B",
	},
	{
		id: "C/C++",
		name: "C/C++",
		description:
			"C/C++ 是嵌入式系统开发中最常用的编程语言,具有高性能和对底层硬件的直接访问能力。",
		icon: "logos:c-plusplus",
		category: "Programming",
		level: "beginner",
		experience: { years: 0, months: 5 },
		projects: ["system-design"

		],
		color: "#00599C",
	},
	{
		id: "视觉",
		name: "视觉",
		description:
			"视觉处理和图像识别技术，用于嵌入式系统中的图像分析和处理。",
		icon: "logos:opencv",
		category: "视觉",
		level: "beginner",
		experience: { years: 0, months: 1 },
		projects: ["system-design"

		],
		color: "#0f9069",
	},
	// Other Skills
	
	{
		id: "cypress",
		name: "Cypress",
		description:
			"A modern end-to-end testing framework for web applications.",
		icon: "logos:cypress-icon",
		category: "other",
		level: "beginner",
		experience: { years: 0, months: 8 },
		projects: ["e2e-testing"],
		color: "#17202C",
	},
];

// Get skill statistics
export const getSkillStats = () => {
	const total = skillsData.length;
	const byLevel = {
		beginner: skillsData.filter((s) => s.level === "beginner").length,
		intermediate: skillsData.filter((s) => s.level === "intermediate")
			.length,
		advanced: skillsData.filter((s) => s.level === "advanced").length,
		expert: skillsData.filter((s) => s.level === "expert").length,
	};
	const byCategory = {
		embedded: skillsData.filter((s) => s.category === "embedded").length,
		Programming: skillsData.filter((s) => s.category === "Programming").length,
		视觉: skillsData.filter((s) => s.category === "视觉").length,
		tools: skillsData.filter((s) => s.category === "tools").length,
		other: skillsData.filter((s) => s.category === "other").length,
	};

	return { total, byLevel, byCategory };
};

// Get skills by category
export const getSkillsByCategory = (category?: string) => {
	if (!category || category === "all") {
		return skillsData;
	}
	return skillsData.filter((s) => s.category === category);
};

// Get advanced skills
export const getAdvancedSkills = () => {
	return skillsData.filter(
		(s) => s.level === "advanced" || s.level === "expert",
	);
};

// Calculate total years of experience
export const getTotalExperience = () => {
	const totalMonths = skillsData.reduce((total, skill) => {
		return total + skill.experience.years * 12 + skill.experience.months;
	}, 0);
	return {
		years: Math.floor(totalMonths / 12),
		months: totalMonths % 12,
	};
};
